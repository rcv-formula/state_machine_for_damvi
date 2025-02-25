// scan_processor_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <deque>
#include <algorithm>

// 2차원 점 구조체 정의
struct Point2D { 
  double x, y; 
};

class ScanProcessor : public rclcpp::Node {
public:
  ScanProcessor() : Node("scan_processor"),
                    tf_buffer_(this->get_clock()),
                    tf_listener_(tf_buffer_) {
    // 스캔 데이터 필터링 파라미터 [m, radian]
    this->declare_parameter<double>("scan_range_min", 0.0);
    this->declare_parameter<double>("scan_range_max", 10.0);
    // YAML 파일에서는 파이 표현식을 사용할 수 없으므로, 코드 내에서 M_PI를 사용합니다.
    this->declare_parameter<double>("scan_angle_min", -M_PI / 3);
    this->declare_parameter<double>("scan_angle_max", M_PI / 3);
    
    // DB clustering 파라미터
    this->declare_parameter<int>("min_cluster_points", 5);
    this->declare_parameter<double>("dbscan_epsilon", 0.5);
    
    // 벽 필터링 관련 파라미터
    this->declare_parameter<double>("wall_distance_threshold", 0.1);
    this->declare_parameter<double>("wall_line_max_error", 0.05);
    this->declare_parameter<int>("min_wall_cluster_points", 5);
    this->declare_parameter<double>("wall_length_threshold", 0.35);
    
    // 멀리 있는 장애물 필터링 관련 파라미터
    this->declare_parameter<double>("far_obstacle_distance_threshold", 5.0); // 3m 이상이면
    this->declare_parameter<int>("far_obstacle_min_points", 3);              // 최소 3개의 점이 필요
    this->declare_parameter<double>("min_obstacle_cluster_density", 4.0);    // 밀도가 낮은 클러스터는 벽 등으로 인해 발생한 값으로 판단하여 제외합니다.
    this->declare_parameter<double>("dynamic_wall_gap_factor", 1.5);         // 먼 거리 벽 그룹화를 위한 동적 임계값 배율
    
    // 파라미터 값 로드
    this->get_parameter("scan_range_min", scan_range_min_);
    this->get_parameter("scan_range_max", scan_range_max_);
    this->get_parameter("scan_angle_min", scan_angle_min_);
    this->get_parameter("scan_angle_max", scan_angle_max_);
    this->get_parameter("min_cluster_points", min_cluster_points_);
    this->get_parameter("dbscan_epsilon", dbscan_epsilon_);
    this->get_parameter("wall_distance_threshold", wall_distance_threshold_);
    this->get_parameter("wall_line_max_error", wall_line_max_error_);
    this->get_parameter("min_wall_cluster_points", min_wall_cluster_points_);
    this->get_parameter("wall_length_threshold", wall_length_threshold_);
    this->get_parameter("far_obstacle_distance_threshold", far_obstacle_distance_threshold_);
    this->get_parameter("far_obstacle_min_points", far_obstacle_min_points_);
    this->get_parameter("min_obstacle_cluster_density", min_obstacle_cluster_density_);
    this->get_parameter("dynamic_wall_gap_factor", dynamic_wall_gap_factor_);
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 20, std::bind(&ScanProcessor::scanCallback, this, std::placeholders::_1));
    candidate_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/obstacle_candidates", 20);
    filtered_scan_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("filtered_scan_points", 20);
  }
  
private:
  // DBSCAN 함수
  std::vector<std::vector<Point2D>> dbscanClustering(const std::vector<Point2D>& points, double eps, int minPts) {
    std::vector<std::vector<Point2D>> clusters;
    const int n = points.size();
    if (n == 0)
      return clusters;

    // 각 점의 방문 여부와 소속 클러스터 ID를 저장 (-1은 아직 할당되지 않음을 의미)
    std::vector<bool> visited(n, false);
    std::vector<int> cluster_ids(n, -1);  

    int cluster_id = 0;

    // 주어진 인덱스의 점과 eps 이내에 있는 모든 점의 인덱스를 반환하는 람다 함수(regionQuery)
    auto regionQuery = [&](int index) -> std::vector<int> {
      std::vector<int> ret;
      for (int i = 0; i < n; i++) {
        double dx = points[index].x - points[i].x;
        double dy = points[index].y - points[i].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= eps)
          ret.push_back(i);
      }
      return ret;
    };

    // DBSCAN 알고리즘 본문
    for (int i = 0; i < n; i++) {
      if (visited[i])
        continue;

      visited[i] = true;
      // 현재 점 i의 이웃 검색
      std::vector<int> neighbors = regionQuery(i);
      
      // 이웃 점의 수가 최소 요구 수보다 작으면, 노이즈(또는 클러스터에 속하지 않는 점)로 간주함
      if (neighbors.size() < static_cast<size_t>(minPts)) {
        // cluster_ids[i]는 그대로 -1로 남겨둠
        continue;
      }
      
      // 새로운 클러스터를 생성하고, 현재 점을 클러스터에 포함시킴
      cluster_ids[i] = cluster_id;
      // 확장을 위해 seeds에 이웃들을 추가
      std::vector<int> seeds = neighbors;
      
      // seeds의 모든 점을 순회하면서 클러스터를 확장함
      for (size_t j = 0; j < seeds.size(); j++) {
        int curr = seeds[j];
        
        if (!visited[curr]) {
          visited[curr] = true;
          std::vector<int> curr_neighbors = regionQuery(curr);
          // 현재 점이 충분한 이웃(코어 포인트)이라면 seeds에 추가하여 클러스터 확장
          if (curr_neighbors.size() >= static_cast<size_t>(minPts)) {
            // 간단한 구현을 위해 중복 여부 검사는 하지 않음
            seeds.insert(seeds.end(), curr_neighbors.begin(), curr_neighbors.end());
          }
        }
        
        // 아직 클러스터에 할당되지 않은 경우 현재 클러스터에 할당
        if (cluster_ids[curr] == -1) {
          cluster_ids[curr] = cluster_id;
        }
      }
      // 클러스터 확장이 끝났으므로, 다음 클러스터 ID로 이동
      cluster_id++;
    }
    
    // cluster_ids를 기반으로 클러스터별로 점들을 모음
    clusters.resize(cluster_id);
    for (int i = 0; i < n; i++) {
      int id = cluster_ids[i];
      if (id != -1) {
        clusters[id].push_back(points[i]);
      }
    }
    return clusters;
  }

  // 연속된 점 그룹(클러스터)이 벽일 가능성이 있는지 판단하는 함수.
  bool isWallCluster(const std::vector<Point2D>& cluster) {
    if (cluster.size() < static_cast<size_t>(min_wall_cluster_points_))
      return false;
    
    const Point2D& p1 = cluster.front();
    const Point2D& p2 = cluster.back();
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double cluster_length = std::sqrt(dx * dx + dy * dy);
    
    // 그룹의 길이가 wall_length_threshold_ 보다 크면 벽으로 판단
    if (cluster_length > wall_length_threshold_)
      return true;
    
    double norm = cluster_length;
    if (norm < 1e-6)
      return false;
    
    double max_error = 0.0;
    // 각 점과 선분 사이의 수선 거리를 계산하여 최대 오차 판단
    for (const auto &pt : cluster) {
      double error = std::fabs(dy * pt.x - dx * pt.y + p2.x * p1.y - p2.y * p1.x) / norm;
      if (error > max_error)
        max_error = error;
    }
    return max_error < wall_line_max_error_;
  }
  
  // *** 새로 추가: 클러스터의 밀도를 계산하고, ROS_INFO로 출력하는 함수 ***  
  double computeAndLogClusterDensity(const std::vector<Point2D>& cluster) {
    double x_min = std::numeric_limits<double>::max();
    double x_max = -std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double y_max = -std::numeric_limits<double>::max();
    for (const auto &pt : cluster) {
      x_min = std::min(x_min, pt.x);
      x_max = std::max(x_max, pt.x);
      y_min = std::min(y_min, pt.y);
      y_max = std::max(y_max, pt.y);
    }
    double extent = std::sqrt((x_max - x_min) * (x_max - x_min) +
                              (y_max - y_min) * (y_max - y_min));
    if (extent < 1e-6)
      extent = 1e-6;  // 0으로 나누는 것을 방지
    double density = static_cast<double>(cluster.size()) / extent;  // 단위: 점/미터
    // RCLCPP_INFO(this->get_logger(), "Cluster density: %.2f, Points: %d, Extent: %.2f", density, static_cast<int>(cluster.size()), extent);
    return density;
  }
  
  // 동적 임계값을 이용하여 벽으로 보이는 점 그룹들을 필터링
  // 센서의 위치(sensor_x, sensor_y)와 스캔의 angle_increment를 사용하여, 멀리 있는 벽의 경우 예상 간격(R * angle_increment)에 따라 인접 점 그룹화를 수행.
  std::vector<Point2D> filterWallPoints(const std::vector<Point2D>& points, double sensor_x, double sensor_y, double angle_increment){
    std::vector<Point2D> filtered_points;
    if (points.empty())
      return filtered_points;
    
    std::vector<Point2D> current_cluster;
    current_cluster.push_back(points[0]);
    
    for (size_t i = 1; i < points.size(); ++i) {
      // 현재 점과 이전 점 사이의 실제 거리 계산
      double dx = points[i].x - points[i - 1].x;
      double dy = points[i].y - points[i - 1].y;
      double actual_gap = std::sqrt(dx * dx + dy * dy);
      
      // 센서로부터 이전 점과 현재 점까지의 거리를 계산하고, 평균값 구함
      double range_prev = std::sqrt(std::pow(points[i - 1].x - sensor_x, 2) + std::pow(points[i - 1].y - sensor_y, 2));
      double range_curr = std::sqrt(std::pow(points[i].x - sensor_x, 2) + std::pow(points[i].y - sensor_y, 2));
      double avg_range = (range_prev + range_curr) / 2.0;
      
      // 예상 점 간격: 멀리 있을수록 각도 해상도에 의한 간격이 커짐
      double expected_gap = avg_range * angle_increment * dynamic_wall_gap_factor_;
      
      // 동적 임계값: 기본 임계값에 예상 gap을 추가
      double dynamic_threshold = wall_distance_threshold_ + expected_gap;
      
      if (actual_gap < dynamic_threshold) {
        current_cluster.push_back(points[i]);
      } else {
        // 그룹이 벽 특성을 띄지 않으면 filtered_points에 추가
        if (!isWallCluster(current_cluster)) {
          filtered_points.insert(filtered_points.end(), current_cluster.begin(), current_cluster.end());
        }
        current_cluster.clear();
        current_cluster.push_back(points[i]);
      }
    }
    // 마지막 그룹 처리
    if (!isWallCluster(current_cluster)) {
      filtered_points.insert(filtered_points.end(), current_cluster.begin(), current_cluster.end());
    }
    return filtered_points;
  }
  // 두 후보점 사이를 선형 보간하여 중간 점들을 생성
  std::vector<geometry_msgs::msg::PointStamped> interpolateCandidates(
      const geometry_msgs::msg::PointStamped& p1,
      const geometry_msgs::msg::PointStamped& p2,
      int num_points)
  {
    std::vector<geometry_msgs::msg::PointStamped> interp;
    for (int i = 1; i <= num_points; i++) {
      double fraction = static_cast<double>(i) / (num_points + 1);
      geometry_msgs::msg::PointStamped new_point;
      new_point.header.frame_id = p1.header.frame_id;
      // stamp는 p1의 시간으로 설정하거나 현재시간/평균시간으로 설정할 수 있음.
      new_point.header.stamp = p1.header.stamp;
      new_point.point.x = p1.point.x + fraction * (p2.point.x - p1.point.x);
      new_point.point.y = p1.point.y + fraction * (p2.point.y - p1.point.y);
      new_point.point.z = 0.0;
      interp.push_back(new_point);
    }
    return interp;
  }
  
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    std::vector<Point2D> points;
    // 1. 포인트 필터링: 메모리 예약
    // 스캔 데이터에서 범위 및 각도 조건에 맞는 점들을 선별
    points.reserve(scan_msg->ranges.size());
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      double range = scan_msg->ranges[i];
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      if (range < scan_range_min_ || range > scan_range_max_ ||
          angle < scan_angle_min_ || angle > scan_angle_max_)
        continue;
      points.push_back({ range * std::cos(angle), range * std::sin(angle) });
    }
    
    // TF 변환: 레이저 좌표계를 map 좌표계로 변환
    std::vector<Point2D> map_points;
    map_points.reserve(points.size());
    // 루프 전에 한 번만 변환 조회
    if (!tf_buffer_.canTransform("map", scan_msg->header.frame_id, scan_msg->header.stamp, tf2::durationFromSec(0.1))) {
      RCLCPP_WARN(this->get_logger(), "Transform not available");
      return;
    }
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform("map", scan_msg->header.frame_id, scan_msg->header.stamp, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Lookup transform failed: %s", ex.what());
      return;
    }    

    for (const auto &pt : points) {
      geometry_msgs::msg::PointStamped laser_pt, map_pt;
      laser_pt.header= scan_msg->header;
      laser_pt.point.x = pt.x;
      laser_pt.point.y = pt.y;
      laser_pt.point.z = 0.0;

      try {
        tf2::doTransform(laser_pt, map_pt, transformStamped);
        map_points.push_back({ map_pt.point.x, map_pt.point.y });
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "doTransform failed: %s", ex.what());
      }
    }
    
    // 센서의 map 좌표계 상 위치를 먼저 구함 (동적 임계값 계산에 사용)
    geometry_msgs::msg::TransformStamped sensor_transform;
    try {
      sensor_transform = tf_buffer_.lookupTransform("map", scan_msg->header.frame_id, scan_msg->header.stamp, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform(Laser Sensor location in map frame) lookup failed: %s", ex.what());
      sensor_transform.transform.translation.x = 0.0;
      sensor_transform.transform.translation.y = 0.0;
    }
    double sensor_x = sensor_transform.transform.translation.x;
    double sensor_y = sensor_transform.transform.translation.y;
    
    // 벽 특성을 보이는 점들을 동적 임계값을 적용하여 필터링 (멀리 있는 벽도 하나의 그룹으로 묶임)
    auto filtered_map_points = filterWallPoints(map_points, sensor_x, sensor_y, scan_msg->angle_increment);
    
    // 필터링된 점들을 rviz에서 확인할 수 있도록 Marker 메시지로 퍼블리시
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = scan_msg->header.stamp;
    marker.ns = "filtered_scan";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // 표시할 점의 크기 (x, y)
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    // 색상 설정 (초록색)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    // 후보군 마커 출력
    for (const auto &pt : filtered_map_points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    filtered_scan_pub_->publish(marker);
    
    // 장애물 후보(클러스터) 검출을 위한 DBSCAN 클러스터링 수행
    auto clusters = dbscanClustering(filtered_map_points, dbscan_epsilon_, min_cluster_points_);
    
    // 각 클러스터에 대해 후보 장애물의 중심을 계산하고,
    // (a) 멀리 있는 장애물의 경우 최소 점 개수 확인,
    // (b) 추가로 클러스터의 점 분포 밀도가 min_obstacle_cluster_density 이상인지 확인
    for (const auto &cluster : clusters) {
      if (cluster.empty())
        continue;
      
      // 클러스터의 중심 계산
      double sum_x = 0, sum_y = 0;
      for (const auto &pt : cluster) {
        sum_x += pt.x;
        sum_y += pt.y;
      }
      geometry_msgs::msg::PointStamped candidate;
      candidate.header.frame_id = "map";
      candidate.header.stamp = scan_msg->header.stamp;
      candidate.point.x = sum_x / cluster.size();
      candidate.point.y = sum_y / cluster.size();
      candidate.point.z = 0.0;
      
      // 센서 위치로부터 후보 중심까지의 거리 계산
      double dx = candidate.point.x - sensor_x;
      double dy = candidate.point.y - sensor_y;
      double candidate_distance = std::sqrt(dx * dx + dy * dy);
      
      // 만약 장애물이 멀리(예: 3m 이상) 있다면, 최소 점 개수(far_obstacle_min_points_)를 만족하는 경우에만 후보로 인정
      if (candidate_distance >= far_obstacle_distance_threshold_ &&
          cluster.size() < static_cast<size_t>(far_obstacle_min_points_)) {
        continue;  // 점의 수가 부족하면 후보로 게시하지 않음.
      }
      
      // 클러스터 점 분포 밀도 계산 및 rosinfo 출력
      double density = computeAndLogClusterDensity(cluster);
      
      // 클러스터 밀도가 최소 밀도(min_obstacle_cluster_density_) 미만이면, 후보로 게시하지 않음.
      if (density < min_obstacle_cluster_density_) {
        continue;
      }
      candidate_pub_->publish(candidate);
      
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr candidate_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filtered_scan_pub_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  double scan_range_min_, scan_range_max_, scan_angle_min_, scan_angle_max_;
  int min_cluster_points_;
  double dbscan_epsilon_;
  double wall_distance_threshold_;
  double wall_line_max_error_;
  int min_wall_cluster_points_;
  double wall_length_threshold_;
  
  double far_obstacle_distance_threshold_;  // 멀리 있는 장애물 판단 임계거리 (예: 3m)
  int far_obstacle_min_points_;               // 멀리 있는 장애물로 인식하기 위한 최소 점의 개수
  
  double min_obstacle_cluster_density_;       // 장애물 클러스터 최소 밀도 (점/미터)
  
  // 동적 임계값 배율 (멀리 있는 벽의 점 간격 보정을 위한 파라미터)
  double dynamic_wall_gap_factor_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}