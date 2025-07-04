#!/usr/bin/env python3
import math, rclpy
from rclpy.node        import Node
from rclpy.duration    import Duration
from nav_msgs.msg      import Odometry, Path
from std_msgs.msg      import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker

class State:
    NORMAL   = 'NORMAL'
    AVOID    = 'AVOID'
    TRAILING = 'TRAILING'

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')

        # ─── 파라미터 ──────────────────────────────────────────
        self.declare_parameter('ttc_thresh',     150.0)   # [s]
        self.declare_parameter('corner_kappa',   0.18)  # [1/m] 
        self.declare_parameter('lookahead_dist', 5.0)   # [m]
        self.ttc_thresh   = self.get_parameter('ttc_thresh').value
        self.corner_kappa = self.get_parameter('corner_kappa').value  
        self.lookahead    = self.get_parameter('lookahead_dist').value

        # ─── 내부 변수 ──────────────────────────────────────────
        self.state = State.NORMAL
        self.ego_pose  = None        # (x,y,yaw)
        self.ego_vel   = None        # (vx,vy)
        self.opp_pose  = None
        self.opp_vel   = None
        self.global_path_msg = self.local_path_msg = None

        self.obstacle_detected = False
        self.last_obs_stamp    = None
        self.obs_timeout       = Duration(seconds=0.5)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL

        )
        self.create_subscription(Odometry,'odom',              self.cb_odom, 10)
        self.create_subscription(Odometry,'opponent_odometry', self.cb_opp , 10)
        self.create_subscription(Path,    'global_path',       self.cb_g   , qos)
        self.create_subscription(Path,    'local_path',        self.cb_l   , 10)
        self.create_subscription(Bool,    'obstacle',          self.cb_obs , 10)

        self.pub_state  = self.create_publisher(String,'state',10)
        self.pub_path   = self.create_publisher(Path  ,'path' , qos)
        self.marker_pub = self.create_publisher(Marker,'state_marker',10)

        self.marker_z_offset = 0.3
        self.timer = self.create_timer(0.05, self.evaluate)     # 20 Hz

    # ───────── 콜백 ────────────────────────────────────────────
    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        self.ego_pose = (p.x, p.y, yaw)
        v = msg.twist.twist.linear
        self.ego_vel  = (v.x, v.y)

    def cb_opp(self, msg: Odometry):
        p = msg.pose.pose.position;  v = msg.twist.twist.linear
        self.opp_pose = (p.x, p.y)
        self.opp_vel  = (v.x, v.y)

    def cb_g  (self, msg: Path): self.global_path_msg = msg
    def cb_l  (self, msg: Path): self.local_path_msg  = msg
    def cb_obs(self, msg: Bool):
        self.obstacle_detected = msg.data
        self.last_obs_stamp    = self.get_clock().now()

    # ───────── 1) TTC 계산 ─────────────────────────────────────
    def compute_ttc(self):
        if None in (self.ego_pose, self.opp_pose, self.ego_vel, self.opp_vel):
            return None
        ex, ey, _ = self.ego_pose
        ox, oy    = self.opp_pose
        dx, dy = ox - ex, oy - ey
        dist = math.hypot(dx, dy)
        if dist < 1e-3: return 0.0
        ux, uy = dx/dist, dy/dist
        rvx = self.ego_vel[0] - self.opp_vel[0]
        rvy = self.ego_vel[1] - self.opp_vel[1]
        closing = rvx*ux + rvy*uy          # + : 접근
        eps = 1e-2
        ttc = dist / (abs(closing) + eps)
        return math.copysign(ttc, closing) # + 접근 / − 이탈

    # ───────── 2) 전방 곡률 최대값 ─────────────────────────────
    def max_kappa_ahead(self):                                        # ★
        if not (self.global_path_msg and self.ego_pose):
            return None
        pts = [(p.pose.position.x, p.pose.position.y) 
               for p in self.global_path_msg.poses]
        if len(pts) < 3: return None
        ex, ey, _ = self.ego_pose
        # ego와 가장 가까운 점 찾기
        idx0 = min(range(len(pts)), key=lambda i:
                   (pts[i][0]-ex)**2 + (pts[i][1]-ey)**2)
        acc = 0.0; kappa_max = 0.0
        for i in range(idx0, len(pts)-2):
            x1,y1 = pts[i]
            x2,y2 = pts[i+1]
            x3,y3 = pts[i+2]
            ds = math.hypot(x2-x1, y2-y1)
            acc += ds
            # 삼각형 면적 기반 곡률
            a = ds
            b = math.hypot(x3-x2, y3-y2)
            c = math.hypot(x3-x1, y3-y1)
            if a*b*c == 0: continue
            area2 = abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1))
            kap = 2*area2 / (a*b*c)
            kappa_max = max(kappa_max, kap)
            if acc >= self.lookahead: break
        return kappa_max if kappa_max > 0 else None
    
    
    def cut_path(self, path:Path) -> Path:
        new_path = Path()
        new_path.header = path.header
        new_path.header.stamp = self.get_clock().now().to_msg()
        dist = 0.0
        prev = None
        for ps in path.poses:
            new_path.poses.append(ps)
            if prev:
                dx = ps.pose.position.x - prev.pose.position.x
                dy = ps.pose.position.y - prev.pose.position.y
                dist += math.hypot(dx,dy)
                if dist >= 5.0:
                    break
            prev = ps

        return new_path

    # ───────── FSM 평가 ───────────────────────────────────────
    def evaluate(self):
        now = self.get_clock().now()
        front = (self.obstacle_detected and
                 self.last_obs_stamp and
                 (now - self.last_obs_stamp) <= self.obs_timeout)

        if not front:
            desired = State.NORMAL
        else:
            corner_flag = False
            kappa_max = self.max_kappa_ahead()                  # ★
            # if kappa_max is not None and kappa_max >= self.corner_kappa:
            #     corner_flag = True
            # if corner_flag:                                     # ★
            #     desired = State.TRAILING
            # else:
            ttc_signed = self.compute_ttc()
            if ttc_signed is not None and ttc_signed > 0 and ttc_signed <= self.ttc_thresh:
                desired = State.TRAILING
            else:
                desired = State.AVOID

        if desired != self.state:
            self.state = desired
            self.get_logger().info(f'STATE → {self.state}')

        # 경로 퍼블리시
        if self.state == State.AVOID and self.local_path_msg:
            cut_path = self.cut_path(self.global_path_msg)
            self.pub_path.publish(cut_path)
        elif self.state == State.TRAILING:
            self.pub_path.publish(self.global_path_msg)
        elif self.global_path_msg is not None:
             self.pub_path.publish(self.global_path_msg)
        else:
            self.get_logger().warn("global_path_msg is None. Not publishing.")


        # 상태 & Marker
        self.pub_state.publish(String(data=self.state))
        self.publish_marker()

        # ─── None-safe 로그 ★
        kappa_log = (self.max_kappa_ahead() or -1.0)
        ttc_log   = (self.compute_ttc()    or -99.0)
        self.get_logger().info(
            f'front={front}  kappa_max={kappa_log:.3f}  TTC={ttc_log:.2f}s')

    # ───────── Marker ─────────────────────────────────────────
    def publish_marker(self):
        if not self.ego_pose: return
        x, y, _ = self.ego_pose
        r,g,b = {'NORMAL':(0.0,1.0,0.0),
                 'AVOID' :(1.0,0.65,0.0),
                 'TRAILING':(1.0,1.0,0.0)}[self.state]
        m = Marker()
        m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg()
        m.ns='fsm'; m.id=0; m.type=m.TEXT_VIEW_FACING; m.action=m.ADD
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=self.marker_z_offset
        m.scale.z=0.3; m.color.r,m.color.g,m.color.b,m.color.a = r,g,b,1.0
        m.text=self.state
        self.marker_pub.publish(m)

def main():
    rclpy.init()
    node = StateMachineNode()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
