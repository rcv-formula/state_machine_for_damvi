#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from scipy.linalg import block_diag
from filterpy.kalman import ExtendedKalmanFilter as EKF
from filterpy.common import Q_discrete_white_noise

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray 

import sys
sys.path.append('/home/rcv/race_stack/cordinate/src')
from cordinate import cordinate_converter

class ObstacleTracker(Node):
    def __init__(self):
        super().__init__('obstacle_tracker')
        self.converter = cordinate_converter(self)
        self.get_logger().info("Obstacle Tracker Initialized")

        self.globalpath_s = np.array([])
        self.globalpath_d = np.array([])
        self.globalpath_v = np.array([])
        self.track_length = None

        self.dt = 0.1

        self.ekf = EKF(dim_x=3, dim_z=3)
        self.ekf.F = np.array([[1, 0, self.dt],
                               [0, 1,    0  ],
                               [0, 0,    1  ]])

        q_s = Q_discrete_white_noise(dim=1, dt=self.dt, var=0.1)  # for s
        q_v = Q_discrete_white_noise(dim=1, dt=self.dt, var=0.1)  # for v
        q_d = np.array([[0.01]])                                # for d

        # 상태 순서를 [s, d, v]
        self.ekf.Q = block_diag(q_s, q_d, q_v)

        # 측정치 z = [s, d, v]
        self.ekf.H = np.eye(3)
        self.ekf.R = np.diag([0.5, 0.5, 0.5])
        self.ekf.P *= 5

        self.obstacle_detected = False
        self.is_initialized = False

        self.obs_x = self.obs_y = self.obs_vx = self.obs_vy = 0.0

        # Subscribers
        self.create_subscription(Bool, "/obstacle", self.obstacle_detected_callback, 10)
        self.create_subscription(Odometry, "/opponent_odom", self.obstacle_callback, 10)

        # Publishers
        self.predicted_obstacle_pub = self.create_publisher(Odometry, "/predicted_obstacle", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/visualization_marker", 10)

        self.timer = self.create_timer(0.1, self.run)

    def obstacle_detected_callback(self, msg):
        self.obstacle_detected = msg.data

    def obstacle_callback(self, msg):
        if not hasattr(msg.pose, 'pose'):
            return
        self.obs_x = msg.pose.pose.position.x
        self.obs_y = msg.pose.pose.position.y
        self.obs_vx = msg.twist.twist.linear.x
        self.obs_vy = msg.twist.twist.linear.y
    
    def global_path(self):
        self.globalpath = self.converter.get_global_path()

        if len(self.globalpath) == 0:
            self.get_logger().warn("Global path is empty!")
            return
        
        output = self.converter.global_to_frenet(self.globalpath)
        self.globalpath_s = np.array([p[0][0] for p in output])
        self.globalpath_d = np.array([p[0][1] for p in output]) 
        self.globalpath_v = np.array([p[1] for p in output]) 

        if self.track_length is None:
            self.track_length = self.converter.get_path_length()

    def find_nearest_global_speed(self, s_obs, d_obs):
        if len(self.globalpath_s) == 0 or len(self.globalpath_v) == 0:
            return 0.0
        distance = np.sqrt((self.globalpath_s - s_obs) ** 2 + (self.globalpath_d - d_obs) ** 2)
    
        idx = np.argmin(distance)  
        return self.globalpath_v[idx]

    @staticmethod
    def normalize_s(s, track_length):
        return s % track_length

    def filter_outlier(self, s_meas, d_meas, threshold_s=1.0, threshold_d=0.5):
        diff_s = abs(self.normalize_s(s_meas - self.ekf.x[0], self.track_length))
        diff_d = abs(d_meas - self.ekf.x[1])
        if diff_s > threshold_s:
            s_meas = self.ekf.x[0]
        if diff_d > threshold_d:
            d_meas = self.ekf.x[1]
        return s_meas, d_meas

    def update(self):
        if self.track_length is None:
            return

        # 측정값 x,y,vx,vy -> s,d,v
        s_meas, d_meas = self.converter.global_to_frenet_point(self.obs_x, self.obs_y)
        s_meas = self.normalize_s(s_meas, self.track_length)
        s_meas, d_meas = self.filter_outlier(s_meas, d_meas, threshold_s=1.0, threshold_d=0.5)

        v_meas = np.hypot(self.obs_vx, self.obs_vy)

        # 측정 벡터 z = [s, d, v]
        z = np.array([s_meas, d_meas, v_meas])
        self.ekf.update(z)
        self.is_initialized = True

    def publish_predicted_obstacle(self):
        obs = Odometry()
        self.global_x, self.global_y = self.converter.frenet_to_global_point(self.ekf.x[0], self.ekf.x[1])
        obs.pose.pose.position.x = self.global_x # global 좌표에서의 x좌표
        obs.pose.pose.position.y = self.global_y # global 좌표에서의 y좌표
        obs.twist.twist.linear.x = self.ekf.x[2] # global 좌표에서의 속도
        self.predicted_obstacle_pub.publish(obs)

    def publish_markers(self):
        markers = MarkerArray()
        if self.is_initialized:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = self.global_x
            marker.pose.position.y = self.global_y
            marker.pose.position.z = 0.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)
    
    def run(self):
        if self.track_length is None:
            self.global_path()

        # obstacle이 측정되지 않을 때 예측값 사용
        # s = s + v*dt 단순 속도 모델 사용
        if not self.obstacle_detected:
            predicted_s = self.ekf.x[0] + self.ekf.x[2] * self.dt
            predicted_d = self.ekf.x[1]
            # ekf.x[2] = 현재 추정 v -> 현재 위치와 가장 가까운 globalpath의 v 로 서서히 변환 
            # ** 0.5는 서서히 변화시키는 파라미터
            predicted_v = self.ekf.x[2] + 0.5 * (self.find_nearest_global_speed(predicted_s, predicted_d) - self.ekf.x[2])
            self.ekf.x = np.array([predicted_s, predicted_d, predicted_v])
        else:
            # obstacle이 측정되면 측정값 그대로 사용
            self.update()

        self.ekf.x[0] = self.normalize_s(self.ekf.x[0], self.track_length)

        self.publish_predicted_obstacle()
        self.publish_markers()

def main():
    rclpy.init()
    tracker = ObstacleTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()