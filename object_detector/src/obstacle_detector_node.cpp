#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

// 상수 정의
constexpr double PI = 3.14159265358979323846;

// 각도를 -PI ~ PI 범위로 normalize 하는 함수
double normalizeAngle(double angle) {
  while (angle > PI)
    angle -= 2.0 * PI;
  while (angle < -PI)
    angle += 2.0 * PI;
  return angle;
}

class ObstacleDetector : public rclcpp::Node {
public:
  ObstacleDetector() : Node("obstacle_detector"),
                      kalman_initialized_(false),
                      has_prev_position_(false),
                      prev_heading_(0.0)
  {
    // DBSCAN 관련 파라미터
    this->declare_parameter<double>("dbscan_eps", 0.3);
    this->declare_parameter<int>("dbscan_min_points", 3);
    this->declare_parameter<bool>("use_weighted_median", false);

    // 칼만 필터 관련 파라미터
    this->declare_parameter<double>("kalman_process_noise", 0.1);
    this->declare_parameter<double>("kalman_measurement_noise", 0.1);
    this->declare_parameter<bool>("use_kalman_filter", true);

    // 장애물 측정 timeout (측정이 이 시간 이상 없으면 장애물 없음으로 간주)
    this->declare_parameter<double>("obstacle_timeout", 1.0);

    // "큐 기반" 데이터 처리를 위한 최소 후보점 수 (기본값은 dbscan_min_points와 동일)
    this->declare_parameter<int>("min_candidates_to_process", 3);

    // 파라미터 취득
    this->get_parameter("dbscan_eps", dbscan_eps_);
    this->get_parameter("dbscan_min_points", dbscan_min_points_);
    this->get_parameter("use_weighted_median", use_weighted_median_);
    this->get_parameter("kalman_process_noise", kalman_process_noise_);
    this->get_parameter("kalman_measurement_noise", kalman_measurement_noise_);
    this->get_parameter("use_kalman_filter", use_kalman_filter_);
    this->get_parameter("obstacle_timeout", obstacle_timeout_);
    this->get_parameter("min_candidates_to_process", min_candidates_to_process_);

    // /opponent_odom 퍼블리셔 (시각화는 별도 노드에서 처리)
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/opponent_odom", 20);
    // 장애물 존재 여부 퍼블리셔
    obstacle_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 20);

    // 장애물 후보점 수신: 큐에 추가한 후, 최소 수치가 되면 즉시 처리
    candidate_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/obstacle_candidates", 20,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        candidate_points_.push_back(*msg);
        // 큐에 누적된 데이터가 최소 수치 이상이면 즉시 처리
        if (candidate_points_.size() >= static_cast<size_t>(min_candidates_to_process_)) {
          processAndPublish();
        }
      });

  }

private:
  // 후보 점들의 평균 timestamp 계산 함수
  rclcpp::Time computeAverageTimestamp(const std::vector<size_t>& indices) {
    uint64_t sum_ns = 0;
    for (size_t idx : indices) {
      const auto &stamp = candidate_points_[idx].header.stamp;
      uint64_t ns = static_cast<uint64_t>(stamp.sec) * 1000000000ull + stamp.nanosec;
      sum_ns += ns;
    }
    uint64_t avg_ns = sum_ns / indices.size();
    builtin_interfaces::msg::Time avg_time;
    avg_time.sec = static_cast<int32_t>(avg_ns / 1000000000ull);
    avg_time.nanosec = static_cast<uint32_t>(avg_ns % 1000000000ull);
    return rclcpp::Time(avg_time);
  }

  // 큐에 누적된 후보 데이터를 DBSCAN 및 칼만 필터 업데이트 후 퍼블리시하는 함수
  void processAndPublish() {
    rclcpp::Time now = this->now();
    double meas_x = 0.0, meas_y = 0.0;
    bool measurement_available = false;
    rclcpp::Time candidate_stamp = now;  // 기본값은 현재 시간

    if (!candidate_points_.empty()) {
      // ----------------------------
      // 1. DBSCAN 클러스터링
      // ----------------------------
      size_t N = candidate_points_.size();
      std::vector<int> cluster_ids(N, -1);  // -1: 미할당, -2: 노이즈

      auto distance = [&](size_t i, size_t j) -> double {
        double dx = candidate_points_[i].point.x - candidate_points_[j].point.x;
        double dy = candidate_points_[i].point.y - candidate_points_[j].point.y;
        return std::sqrt(dx * dx + dy * dy);
      };

      auto regionQuery = [&](size_t i) -> std::vector<size_t> {
        std::vector<size_t> neighbors;
        for (size_t j = 0; j < N; j++) {
          if (distance(i, j) <= dbscan_eps_) {
            neighbors.push_back(j);
          }
        }
        return neighbors;
      };

      int cluster_id = 0;
      for (size_t i = 0; i < N; i++) {
        if (cluster_ids[i] != -1)
          continue;
        auto neighbors = regionQuery(i);
        if (neighbors.size() < static_cast<size_t>(dbscan_min_points_)) {
          cluster_ids[i] = -2;  // 노이즈로 분류
          continue;
        }
        // 새로운 클러스터 생성
        cluster_ids[i] = cluster_id;
        std::vector<size_t> seed_set = neighbors;
        for (size_t idx = 0; idx < seed_set.size(); idx++) {
          size_t j = seed_set[idx];
          if (cluster_ids[j] == -2)
            cluster_ids[j] = cluster_id;
          if (cluster_ids[j] != -1)
            continue;
          cluster_ids[j] = cluster_id;
          auto neighbors_j = regionQuery(j);
          if (neighbors_j.size() >= static_cast<size_t>(dbscan_min_points_)) {
            seed_set.insert(seed_set.end(), neighbors_j.begin(), neighbors_j.end());
          }
        }
        cluster_id++;
      }

      // ----------------------------
      // 2. 가장 큰 클러스터 선택
      // ----------------------------
      std::vector<std::vector<size_t>> clusters(cluster_id);
      for (size_t i = 0; i < N; i++) {
        if (cluster_ids[i] >= 0)
          clusters[cluster_ids[i]].push_back(i);
      }
      int best_cluster = -1;
      size_t best_cluster_size = 0;
      for (int i = 0; i < cluster_id; i++) {
        if (clusters[i].size() > best_cluster_size) {
          best_cluster_size = clusters[i].size();
          best_cluster = i;
        }
      }

      if (best_cluster != -1 && best_cluster_size > 0) {
        measurement_available = true;
        // 측정이 이루어진 경우, 마지막 측정 시간 업데이트
        last_measurement_time_ = now;
        candidate_stamp = computeAverageTimestamp(clusters[best_cluster]);

        // ----------------------------
        // 3. 대표점 산출: 가중 평균 또는 가중 중앙값
        // ----------------------------
        double sum_x = 0.0, sum_y = 0.0;
        for (size_t idx : clusters[best_cluster]) {
          sum_x += candidate_points_[idx].point.x;
          sum_y += candidate_points_[idx].point.y;
        }
        double center_x = sum_x / clusters[best_cluster].size();
        double center_y = sum_y / clusters[best_cluster].size();
        const double epsilon = 1e-3;

        if (!use_weighted_median_) {
          double weighted_sum_x = 0.0, weighted_sum_y = 0.0, total_weight = 0.0;
          for (size_t idx : clusters[best_cluster]) {
            double dx = candidate_points_[idx].point.x - center_x;
            double dy = candidate_points_[idx].point.y - center_y;
            double d = std::sqrt(dx * dx + dy * dy);
            double weight = 1.0 / (d + epsilon);
            weighted_sum_x += candidate_points_[idx].point.x * weight;
            weighted_sum_y += candidate_points_[idx].point.y * weight;
            total_weight += weight;
          }
          meas_x = weighted_sum_x / total_weight;
          meas_y = weighted_sum_y / total_weight;
        } else {
          struct WeightedVal {
            double val;
            double weight;
          };
          std::vector<WeightedVal> wx, wy;
          double total_weight = 0.0;
          for (size_t idx : clusters[best_cluster]) {
            double dx = candidate_points_[idx].point.x - center_x;
            double dy = candidate_points_[idx].point.y - center_y;
            double d = std::sqrt(dx * dx + dy * dy);
            double weight = 1.0 / (d + epsilon);
            wx.push_back({candidate_points_[idx].point.x, weight});
            wy.push_back({candidate_points_[idx].point.y, weight});
            total_weight += weight;
          }
          auto cmp = [](const WeightedVal &a, const WeightedVal &b) { return a.val < b.val; };
          std::sort(wx.begin(), wx.end(), cmp);
          std::sort(wy.begin(), wy.end(), cmp);
          double cum = 0.0, median_x = wx.front().val;
          for (const auto &w : wx) {
            cum += w.weight;
            if (cum >= total_weight / 2.0) {
              median_x = w.val;
              break;
            }
          }
          cum = 0.0;
          double median_y = wy.front().val;
          for (const auto &w : wy) {
            cum += w.weight;
            if (cum >= total_weight / 2.0) {
              median_y = w.val;
              break;
            }
          }
          meas_x = median_x;
          meas_y = median_y;
        }
      }
      // 후보점 큐 초기화 (중복 처리를 방지)
      candidate_points_.clear();
    }
    
    // ----------------------------
    // 4. 칼만 필터 업데이트 (측정이 없으면 예측만 수행)
    // ----------------------------
    if (use_kalman_filter_) {
      double dt = kalman_initialized_ ? (now - last_kf_time_).seconds() : 0.0;
      if (kalman_initialized_) {
        // 예측 단계: 이전 속도를 이용해 상태 업데이트
        kf_state_[0] += kf_state_[2] * dt;
        kf_state_[1] += kf_state_[3] * dt;
        for (int i = 0; i < 4; i++) {
          kf_P_[i][i] += kalman_process_noise_;
        }
      } else {
        if (measurement_available) {
          // 첫 측정 시 칼만 필터 초기화
          kf_state_[0] = meas_x;
          kf_state_[1] = meas_y;
          kf_state_[2] = 0.0;
          kf_state_[3] = 0.0;
          for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
              kf_P_[i][j] = (i == j) ? 1.0 : 0.0;
            }
          }
          kalman_initialized_ = true;
          last_kf_time_ = now;
        }
      }
      
      if (measurement_available && kalman_initialized_) {
        double z[2] = { meas_x, meas_y };
        double y[2] = { z[0] - kf_state_[0], z[1] - kf_state_[1] };
        double S[2][2] = {
          { kf_P_[0][0] + kalman_measurement_noise_, kf_P_[0][1] },
          { kf_P_[1][0], kf_P_[1][1] + kalman_measurement_noise_ }
        };
        double invS0 = 1.0 / S[0][0];
        double invS1 = 1.0 / S[1][1];
        double K[4][2];
        for (int i = 0; i < 4; i++) {
          K[i][0] = kf_P_[i][0] * invS0;
          K[i][1] = kf_P_[i][1] * invS1;
        }
        for (int i = 0; i < 4; i++) {
          kf_state_[i] += K[i][0] * y[0] + K[i][1] * y[1];
        }
        kf_P_[0][0] = (1 - K[0][0]) * kf_P_[0][0];
        kf_P_[1][1] = (1 - K[1][1]) * kf_P_[1][1];
        last_kf_time_ = now;
      }
    } else {
      if (measurement_available) {
        kf_state_[0] = meas_x;
        kf_state_[1] = meas_y;
        kf_state_[2] = 0.0;
        kf_state_[3] = 0.0;
        kalman_initialized_ = true;
        last_kf_time_ = now;
      }
    }

    // ----------------------------
    // 5. 최종 장애물 상태 퍼블리시 (칼만 필터 상태 기반)
    // ----------------------------
    if (kalman_initialized_) {
      publishOdomWithKalmanState(candidate_stamp);
    }

    // ----------------------------
    // 6. 장애물 감지 상태(bool) 퍼블리시
    // ----------------------------
    std_msgs::msg::Bool detected_msg;
    detected_msg.data = (kalman_initialized_ && (now - last_measurement_time_).seconds() < obstacle_timeout_);
    obstacle_detected_pub_->publish(detected_msg);

    // 처리 종료 시간 기록 및 소요 시간 로그 출력
    rclcpp::Time end_time = this->now();
    double processing_duration = (end_time - now).seconds();
    //RCLCPP_INFO(this->get_logger(), "Processing time: %.6f seconds", processing_duration);

  }

  // 칼만 필터 상태를 이용해 odom 메시지를 퍼블리시 (orientation은 이전 위치와 현재 위치 차이로 계산)
  void publishOdomWithKalmanState(const rclcpp::Time & stamp) {
    double heading = 0.0;
    if (has_prev_position_) {
      double dx = kf_state_[0] - prev_x_;
      double dy = kf_state_[1] - prev_y_;
      if (std::sqrt(dx * dx + dy * dy) > 1e-3) {
        heading = std::atan2(dy, dx);
      } else {
        heading = prev_heading_;
      }
    } else {
      heading = 0.0;
      has_prev_position_ = true;
    }
    prev_x_ = kf_state_[0];
    prev_y_ = kf_state_[1];
    prev_heading_ = heading;

    double sin_yaw = std::sin(heading * 0.5);
    double cos_yaw = std::cos(heading * 0.5);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "map";  // map 좌표계

    odom_msg.pose.pose.position.x = kf_state_[0];
    odom_msg.pose.pose.position.y = kf_state_[1];
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.twist.twist.linear.x = kf_state_[2];
    odom_msg.twist.twist.linear.y = kf_state_[3];
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin_yaw;
    odom_msg.pose.pose.orientation.w = cos_yaw;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom_msg);

    //RCLCPP_INFO(this->get_logger(), "KF state: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, heading=%.2f",
    //            kf_state_[0], kf_state_[1], kf_state_[2], kf_state_[3], heading);
  }

  // 멤버 변수들
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr candidate_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub_;
  
  // 후보점을 누적하는 큐
  std::vector<geometry_msgs::msg::PointStamped> candidate_points_;

  // DBSCAN 파라미터
  double dbscan_eps_;
  int dbscan_min_points_;
  bool use_weighted_median_;

  // 큐 기반 데이터 처리를 위한 최소 후보점 수
  int min_candidates_to_process_;

  // 칼만 필터 관련 멤버 변수
  bool kalman_initialized_;
  // 상태: [x, y, vx, vy]
  double kf_state_[4];
  // 공분산 (4x4)
  double kf_P_[4][4];
  double kalman_process_noise_;
  double kalman_measurement_noise_;
  rclcpp::Time last_kf_time_;

  // 칼만 필터 적용 여부 파라미터
  bool use_kalman_filter_;

  // 이전 위치 및 heading 저장 (orientation 계산용)
  double prev_x_;
  double prev_y_;
  double prev_heading_;
  bool has_prev_position_;

  // 마지막 측정 시간 및 timeout (초)
  rclcpp::Time last_measurement_time_;
  double obstacle_timeout_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
