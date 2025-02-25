from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class cordinate_converter:
    def __init__(self, node :Node):
        self.node = node

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,               # 신뢰성: Reliable
            durability=DurabilityPolicy.TRANSIENT_LOCAL,          # 지속성: Transient Local
            history=HistoryPolicy.KEEP_LAST,                      # 히스토리: Keep Last
            depth=1                                              # Depth 설정
        )
        self.path_recived = False
        self.global_path = list()


        self.subscription = self.node.create_subscription(Path, 'global_path', self.path_callback, qos_profile)
        
        timeout = 5.0  # 최대 5초 대기
        start_time = self.node.get_clock().now().nanoseconds / 1e9
        while not self.path_recived and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current_time = self.node.get_clock().now().nanoseconds / 1e9
            if current_time - start_time > timeout:
                self.node.get_logger().warn("Timeout waiting for global path!")
                break
        self.node.get_logger().info(f"path has recived, total {len(self.global_path)} points")

    def get_path_length(self):
        return self._calc_path_distance(0,len(self.global_path)-1)# len(self.global_path)

    def get_global_path(self):
        return self.global_path

    def _get_closest_index(self, x, y):
        idx = 0
        closest_dist = self._calc_distance([x, y], self.global_path[0][:2])
        for i in range (1, len(self.global_path),1):
            dist = self._calc_distance([x, y], self.global_path[i][:2])
            if dist < closest_dist:
                idx = i
                closest_dist = dist
        return idx
    
    def global_to_frenet_point(self, x, y):
        closest_idx = self._get_closest_index(x, y)
        out1 = self._calc_proj(closest_idx, closest_idx+1, x, y)
        out2 = self._calc_proj(closest_idx-1, closest_idx, x, y)

        s=0
        d=0
        self.node.get_logger().info(f"index : {closest_idx} s : { out1[0]}, d : {out1[1]}")
        if abs(out1[1]) > abs(out2[1]):
            s = out2[0] + self._calc_path_distance(0, closest_idx-1)
            d = out2[1]
        else:
            s = out1[0] + self._calc_path_distance(0, closest_idx)
            d = out1[1]
        return [s, d]
    
    def global_to_frenet(self, path_list:list):
        output = []
        for i in range (0, len(path_list)):
            output.append([self.global_to_frenet_point(path_list[i][0], path_list[i][1])+ [path_list[i][2]]])
        return output
    
    def frenet_to_global_point(self, s, d):
        if(s<0):
            s = s+self.get_path_length()
        s = s % self.get_path_length()

        start_idx = self._get_start_path_from_frenet(s)
        next_idx = (start_idx+1)%len(self.global_path)
        s = d-self.get_path_length(start_idx)

        start_point = np.array([self.global_path[start_idx][0], self.global_path[start_idx][1]])
        next_point = np.array([self.global_path[next_idx][0], self.global_path[next_idx][1]])

        path_u_vector = (next_point - start_point)
        projPoint = start_point + (path_u_vector*s)
        normalVector = self.rotate_right_90(path_u_vector)
        global_point = projPoint + (normalVector * d)
        return list(global_point)

    def frenet_to_global(self, path_list : list):
        output=[]
        for i in range (0, len(path_list)):
            output.append(self.frenet_to_global_point(path_list[0], path_list[1])+[path_list[2]])
        return output

    def rotate_right_90(v):
        return np.array([v[1], -v[0]])

        
    def _get_start_path_from_frenet(self, s):
        idx = 0
        while self._calc_path_distance(0, idx +1)<=s:
            idx +=1
        return idx

    def _calc_proj(self, idx, next_idx, x, y):
        path_size = len(self.global_path)      
        idx = (idx%path_size + path_size)%path_size
        next_idx = (next_idx%path_size + path_size)%path_size

        self.node.get_logger().info(f"index : {idx}")
        
        pointA = np.array(self.global_path[idx][:2])
        pointB = np.array(self.global_path[next_idx][:2])
        pointC = np.array([x,y])
        print(pointA)
        print(pointB)

        vectorA = pointB - pointA
        vectorB = pointC - pointA

        print(vectorA)
        print(vectorB)

        proj_t = np.dot(vectorB, vectorA)/np.dot(vectorA, vectorA)
        proj_point = pointA + (proj_t * vectorA)

        d = np.linalg.norm(proj_point - pointC)
        if np.cross(vectorA, vectorB)>0:
            d = -d
        s = proj_t * np.dot(vectorA, vectorA)
        output = [s, d]
        return output

    def _calc_path_distance(self, start_idx, end_idx):
        distance_counter =0
        if end_idx<start_idx:
            end_idx = end_idx + len(self.global_path)
        for i in range (start_idx, end_idx):
            distance_counter = distance_counter+self._calc_path_to_path_distant(i)
        return distance_counter
    
    def _calc_path_to_path_distant(self, idx):
        if idx < 0:
            idx = idx + len(self.global_path)
        idx = idx % len(self.global_path)
        next_idx = (idx+1)%len(self.global_path)

        cur = [self.global_path[idx][0], self.global_path[idx][1]]
        next = [self.global_path[next_idx][0], self.global_path[next_idx][1]]
        return self._calc_distance(cur, next)

    def _calc_distance(self, A:list, B:list):
        return math.sqrt(math.pow(A[0]-B[0], 2 )+ math.pow( A[1]-B[1], 2))   

    def path_callback(self, msg : Path):
        self.global_path = list()
        for pose in msg.poses:
            self.global_path.append([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        self.path_recived = True
    




def main():
    rclpy.init()
    node = Node("cordinate_converter_test_node")
    converter = cordinate_converter(node)
    # 스피닝이 이미 호출되었으므로, 여기서도 spin_once() 호출 가능
    #path_length = converter.get_path_length()
    #node.get_logger().info(f"Path length is :{path_length}")
    #cord = converter.global_to_frenet(0.1134318,5.2239407)
    #node.get_logger().info(f"frenet frame : s {cord[0]}, d {cord[1]}")
    #node.destroy_node()
    print(converter.get_global_path())
    rclpy.shutdown()

if __name__ == '__main__':
    main()