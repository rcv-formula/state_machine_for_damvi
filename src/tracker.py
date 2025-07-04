#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class OpponentUtils:
    def __init__(self):
        self.p = 0.5
        self.v = 0.8
        self.vel_threshold = 0.05  # m/s
        self.match = 2.0
        self.tuning_window = deque(maxlen=20)

    def calc_distance(self, o1, o2, _):
        dp = np.hypot(o1['x'] - o2['x'], o1['y'] - o2['y'])
        dv = np.hypot(o1.get('vx', 0.0) - o2.get('vx', 0.0),
                       o1.get('vy', 0.0) - o2.get('vy', 0.0))
        return self.p * dp + self.v * dv

    def is_static(self, o):
        raw_speed = np.hypot(o.get('raw_vx', 0.0), o.get('raw_vy', 0.0))
        rel_speed = np.hypot(o.get('vx', 0.0), o.get('vy', 0.0))
        thr = self.vel_threshold
        return raw_speed <= thr or rel_speed <= thr

    def verify_opponent(self, dyn, _, prev):
        if not dyn:
            return None
        if prev is None:
            return max(dyn, key=lambda o: np.hypot(o.get('vx', 0.0), o.get('vy', 0.0)))
        best, best_score = None, float('inf')
        for c in dyn:
            score = self.calc_distance(c, prev, None)
            if score < best_score:
                best_score, best = score, c
        return best if best_score < self.match else None

    def update_tuning(self, matched, count, obs):
        self.tuning_window.append((matched, count))
        failure = 1 - sum(m for m, _ in self.tuning_window) / len(self.tuning_window)
        if failure > 0.7:
            self.match = min(self.match + 0.1, 5.0)
            self.vel_threshold = min(self.vel_threshold + 0.05, 1.0)
        elif failure < 0.2:
            self.match = max(self.match - 0.05, 1.0)
            self.vel_threshold = max(self.vel_threshold - 0.05, 0.1)
        print(f"[TUNING] failure={failure:.2f}, match={self.match:.2f}, vel_thr={self.vel_threshold:.2f}")

class Classifier(Node):
    def __init__(self):
        super().__init__('opponent_tracker_node')


        self.create_subscription(Bool, '/obstacle_detected', self.obs_cb, 10)
        self.create_subscription(Odometry, '/opponent_odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.odom_pub     = self.create_publisher(Odometry, '/opponent_odometry', 10)
        self.marker_pub   = self.create_publisher(Marker, '/opponent_marker', 10)
        self.detected_pub = self.create_publisher(Bool, '/obstacle', 10)

        self.odom_buffer = deque()
        self.prev_opp = None
        self.opp_utils = OpponentUtils()

        self.ego_vel = (0.0, 0.0)
        self.ego_yaw_rate = 0.0
        self.ego_pose = (0.0, 0.0)

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.frame_dt = 0.1
        self.last_pub_pos = None
        self.static_timeout = 1.0
        self.obstacle = False

    def obs_cb(self, msg: Bool):
        self.obstacle = bool(msg.data)

    def odom_cb(self, msg: Odometry):
        self.odom_buffer.append({
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y
        })
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_time >= self.frame_dt:
            self.process_frame()
            self.last_time = now

    def cb_odom(self, msg: Odometry):
        self.ego_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.ego_yaw_rate = msg.twist.twist.angular.z
        self.ego_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def process_frame(self):
        if not self.odom_buffer:
            return
        candidates = list(self.odom_buffer)
        self.odom_buffer.clear()

        for i in range(1, len(candidates)):
            prev, curr = candidates[i-1], candidates[i]
            raw_vx = (curr['x'] - prev['x']) / self.frame_dt
            raw_vy = (curr['y'] - prev['y']) / self.frame_dt
            dx = curr['x'] - self.ego_pose[0]
            dy = curr['y'] - self.ego_pose[1]
            rot_vx = -self.ego_yaw_rate * dy
            rot_vy =  self.ego_yaw_rate * dx
            curr['vx'] = raw_vx - self.ego_vel[0] - rot_vx
            curr['vy'] = raw_vy - self.ego_vel[1] - rot_vy
            curr['raw_vx'] = raw_vx
            curr['raw_vy'] = raw_vy

        first = candidates[0]
        first.update({'vx': 0.0, 'vy': 0.0, 'raw_vx': 0.0, 'raw_vy': 0.0})

        dyn = []
        for c in candidates:
            dx = c['x'] - self.ego_pose[0]
            dy = c['y'] - self.ego_pose[1]
            dist = np.hypot(dx,dy)

            if dist <2.0 or not self.opp_utils.is_static(c):
                dyn.append(c)
                
        opp = self.opp_utils.verify_opponent(dyn, None, self.prev_opp)
        matched = opp is not None
        self.opp_utils.update_tuning(matched, len(candidates), self.obstacle)

        if matched:
            self.prev_opp = opp
            self.publish_odometry(opp)
            self.publish_marker(opp)
            self.publish_detected(opp)

            now = self.get_clock().now().nanoseconds / 1e9
            moved = (self.last_pub_pos is None or
                     np.hypot(opp['x'] - self.last_pub_pos[0], opp['y'] - self.last_pub_pos[1]) > 1e-3)
            if moved or now - self.last_time > self.static_timeout:
                self.publish_odometry(opp)
                self.publish_marker(opp)
                self.publish_detected(opp)
                self.last_pub_pos = (opp['x'], opp['y'])
        else:
            self.prev_opp = None
            self.get_logger().info('Opponent lost')

    def publish_odometry(self, o):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = o['x']
        msg.pose.pose.position.y = o['y']
        speed = np.hypot(o.get('vx', 0.0), o.get('vy', 0.0))
        if speed > self.opp_utils.vel_threshold:
            yaw = np.arctan2(o['vy'], o['vx'])
            self.last_heading = yaw
        else:
            yaw = getattr(self, 'last_heading', 0.0)
        msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        msg.twist.twist.linear.x = o['vx']
        msg.twist.twist.linear.y = o['vy']
        self.odom_pub.publish(msg)

    def publish_marker(self, o):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'opponent'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = o['x']
        m.pose.position.y = o['y']
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r = 1.0
        m.color.a = 1.0
        self.marker_pub.publish(m)

    def publish_detected(self, opp):
        msg = Bool()
        dx = opp['x'] - self.ego_pose[0]
        dy = opp['y'] - self.ego_pose[1]
        msg.data = bool(np.hypot(dx, dy) < 2.0)
        self.detected_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = Classifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
