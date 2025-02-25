#include "cordinate.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

CordinateConverter::CordinateConverter(rclcpp::Node::SharedPtr node,
                                       bool node_mode = false,
                                       std::string config_file_path = "",
                                       const std::string path_type = "ring") {
  node_ = node;
  node_mode_ = node_mode;

  if (node_mode_) {
    if (config_file_path == "") {
      config_file_path =
          ament_index_cpp::get_package_share_directory("cordinate") +
          "/config/config.yaml";
    }
    std::string path_file_path, pgm_file_path, map_yaml_file_path;

    std::ifstream file(config_file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(node->get_logger(), "cannot open path file : %s",
                   config_file_path.c_str());
    } else {
      std::string yaml_line;
      while (std::getline(file, yaml_line)) {
        std::istringstream ss(yaml_line);
        std::string key;
        ss >> key;
        if (key == "path:") {
          ss >> path_file_path;
        } else if (key == "map_yaml:") {
          ss >> map_yaml_file_path;
        } else if (key == "map_pgm:") {
          ss >> pgm_file_path;
        }
      }

      readPath(path_file_path);
      path_msg_generator();

      file.close();
      map_node_ =
          new OccupancyGridNode(node_, pgm_file_path, map_yaml_file_path);
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local();
      qos.reliable();

      publisher_ =
          node_->create_publisher<nav_msgs::msg::Path>("global_path", qos);
      timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
        path_publisher();
        timer_->cancel();
      });
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "waiting for global path topic");
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();
    subscriber_ = node_->create_subscription<nav_msgs::msg::Path>(
        "global_path", qos, [this](const nav_msgs::msg::Path::SharedPtr msg) {
          pathCallback(*msg);
        });
  }
}

CordinateConverter::~CordinateConverter() {
  if (node_mode_) {
    delete map_node_;
  }
}

void CordinateConverter::readPath(std::string path_file_path) {
  std::ifstream path_file(path_file_path);
  std::string line_temp;
  while (getline(path_file, line_temp)) {
    std::stringstream ss(line_temp);
    std::string cell;
    std::vector<double> row;
    int col_counter = 0;
    while (getline(ss, cell, ',') && col_counter < 3) {
      row.push_back(stod(cell));
    }
    global_path_.push_back(row);
  }

  RCLCPP_INFO(node_->get_logger(),
              "successfully load csv path file, %ld points from %s",
              global_path_.size(), path_file_path.c_str());
  for (int i = 0; i < global_path_.size(); i++) {
    global_path_heading_.push_back(calcPathToPathHeading(i));
    global_path_distance_.push_back(calcPathToPathDistance(i));
    global_path_reaching_time_.push_back(calcPathToPathRechingTime(i));
  }
}

void CordinateConverter::readPath(nav_msgs::msg::Path path_topic) {
  for (int i = 0; i < path_topic.poses.size(); i++) {
    std::vector<double> temp;
    temp.push_back(path_topic.poses[i].pose.position.x);
    temp.push_back(path_topic.poses[i].pose.position.y);
    temp.push_back(path_topic.poses[i].pose.position.z);
    global_path_.push_back(temp);
  }
  for (int i = 0; i < global_path_.size(); i++) {
    global_path_heading_.push_back(calcPathToPathHeading(i));
    global_path_distance_.push_back(calcPathToPathDistance(i));
    global_path_reaching_time_.push_back(calcPathToPathRechingTime(i));
  }
}

double CordinateConverter::calcPathToPathHeading(int idx) {
  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

  double dx = next_x - cur_x, dy = next_y - cur_y;
  double theta = atan2(dy, dx);
  return theta;
}

double CordinateConverter::calcPathToPathDistance(int idx) {

  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

  return calcDistance(cur_x, cur_y, next_x, next_y);
}

double CordinateConverter::calcPathToPathRechingTime(int idx) {
  idx = idx % global_path_.size();
  int prev_idx = ((idx - 1) + global_path_.size()) % global_path_.size();
  double speed = global_path_[idx][2], prev_speed = global_path_[prev_idx][2];
  double avg_speed = (speed + prev_speed) / 2;
  return calcPathToPathDistance(prev_idx) / avg_speed;
}

int CordinateConverter::getClosestsIndex(double x, double y) {
  double closest_dist = INFINITY;
  int idx;
  for (int i = 0; i < global_path_.size(); i++) {
    double dist = calcDistance(global_path_[i][0], global_path_[i][1], x, y);
    if (closest_dist > dist) {
      closest_dist = dist;
      idx = i;
    }
  }
  return idx;
}

std::vector<double> CordinateConverter::globalToFrenet(double x, double y) {
  int closest_idx = getClosestsIndex(x, y);
  std::pair<double, double> out1 = calcDS(closest_idx, closest_idx + 1, x, y);
  std::pair<double, double> out2 = calcDS(closest_idx - 1, closest_idx, x, y);
  double dist;
  double s;

  if (std::abs(out1.first) > std::abs(out2.first)) {
    dist = out2.first;
    s = out2.second + calcPathDistance(0, closest_idx);
  } else {
    dist = out1.first;
    s = out1.second + calcPathDistance(0, closest_idx);
  }
  std::vector<double> output;
  output.push_back(s);
  output.push_back(dist);
  return output;
}

double CordinateConverter::calcPathDistance(int idx_start, int idx_end) {
  double distance_counter = 0;
  if (idx_end < idx_start) {
    idx_end += global_path_.size();
  }
  for (int idx = idx_start; idx < idx_end; idx++) {
    distance_counter += calcPathToPathDistance(idx);
  }
  return distance_counter;
}

double CordinateConverter::calcDistance(double x, double y, double x1,
                                        double y1) {
  return sqrt(pow(x - x1, 2) + pow(y - y1, 2));
}

double CordinateConverter::getpathLenth() {
  return calcPathDistance(0, global_path_.size() - 1);
}

void CordinateConverter::path_publisher() {
  publisher_->publish(*global_path_msg_);
  RCLCPP_INFO(node_->get_logger(), "Published Path with %zu points",
              global_path_msg_->poses.size());
}

void CordinateConverter::path_msg_generator() {
  global_path_msg_ = std::make_shared<nav_msgs::msg::Path>();
  global_path_msg_->header.stamp = node_->get_clock()->now();
  global_path_msg_->header.frame_id = "map";

  for (int i = 0; i < global_path_.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = global_path_[i][0];
    pose.pose.position.y = global_path_[i][1];
    pose.pose.position.z = global_path_[i][2];
    pose.pose.orientation.z = global_path_heading_[i];
    pose.pose.orientation.w = 1.0; // 단순한 예제이므로 회전 없음
    global_path_msg_->poses.push_back(pose);
  }
  global_path_msg_;
}

void CordinateConverter::pathCallback(nav_msgs::msg::Path msg) {
  global_path_msg_ = std::make_shared<nav_msgs::msg::Path>(msg);
}

std::vector<std::vector<double>> CordinateConverter::getGlobalPath() {
  return global_path_;
}

std::pair<double, double> CordinateConverter::calcDS(int idx, int next_idx,
                                                     double x, double y) {
  // 인덱스가 음수가 될 경우를 고려하여 올바르게 보정합니다.
  int n = global_path_.size();
  idx = (idx % n + n) % n;
  next_idx = (next_idx % n + n) % n;

  double Ax = global_path_[idx][0], Ay = global_path_[idx][1];
  double Bx = global_path_[next_idx][0], By = global_path_[next_idx][1];

  // 벡터 AB 구하기
  double dx = Bx - Ax;
  double dy = By - Ay;
  double segment_length_sq = dx * dx + dy * dy;
  if (segment_length_sq < 1e-6) { // 두 점이 거의 같은 경우
    return std::make_pair(0.0, 0.0);
  }

  // 점 A에서 점 P=(x,y)까지의 벡터와 AB 벡터의 내적을 이용해 투영 인자 t 계산
  double t = ((x - Ax) * dx + (y - Ay) * dy) / segment_length_sq;

  // 투영점 좌표 계산: A + t*(B-A)
  double proj_x = Ax + t * dx;
  double proj_y = Ay + t * dy;

  // 점 P와 투영점 사이의 유클리드 거리 (절대값)
  double d =
      std::sqrt((x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y));

  // 교차곱(cross product)을 이용해 d의 부호 결정:
  // AB x AP의 z성분이 음수면 오른쪽, 양수면 왼쪽으로 간주
  double cross = dx * (y - Ay) - dy * (x - Ax);
  if (cross > 0) {
    d = -d;
  }

  // 종방향 거리 s: A부터 투영점까지의 거리 (t*segment_length)
  double segment_length = std::sqrt(segment_length_sq);
  double s = t * segment_length;

  return std::make_pair(d, s);
}

std::vector<double> CordinateConverter::FrenetToGlobal(double s, double d) {
  if (s < 0)
    s += getpathLenth();
  s = std::fmod(s, getpathLenth());
  int start_idx = getStartPathFromFrenet(s, d);
  int next_idx = (start_idx + 1) % global_path_.size();
  std::vector<double> start_path_point = {global_path_[start_idx][0],
                                          global_path_[start_idx][1]};
  std::vector<double> next_path_point = {global_path_[next_idx][0],
                                         global_path_[next_idx][1]};
  std::vector<double> path_vector =
      pointToVector(start_path_point, next_path_point);
  double single_path_lenth = std::sqrt(dotProudct(path_vector, path_vector));
  double path_to_projPoint_lenth = (s - calcPathDistance(0, start_idx));
  std::vector<double> projPoint = vectorAdd(
      vectorScalarMultiple(vectorScalarDivision(path_vector, single_path_lenth),
                           path_to_projPoint_lenth),
      start_path_point);
  std::vector<double> normalVector =
      toNormal2DVector(vectorScalarDivision(path_vector, single_path_lenth));
  std::vector<double> global_point =
      vectorAdd(projPoint, vectorScalarMultiple(normalVector, d));
  return global_point;
}

std::vector<double>
CordinateConverter::toNormal2DVector(std::vector<double> vector) {
  std::vector<double> output = {vector[1], vector[0] * -1};
  return output;
}

std::vector<double> vectorSubtract(std::vector<double> vectorA,
                                   std::vector<double> vectorB) {
  std::vector<double> output;
  for (int i = 0; i < std::min(vectorA.size(), vectorB.size()); i++) {
    output.push_back(vectorA[i] - vectorB[i]);
  }
  return output;
}

std::vector<double> CordinateConverter::vectorAdd(std::vector<double> vectorA,
                                                  std::vector<double> vectorB) {
  std::vector<double> output;
  for (int i = 0; i < std::min(vectorA.size(), vectorB.size()); i++) {
    output.push_back(vectorA[i] + vectorB[i]);
  }
  return output;
}

std::vector<double>
CordinateConverter::vectorScalarMultiple(std::vector<double> vector, double a) {
  for (int i = 0; i < vector.size(); i++) {
    vector[i] *= a;
  }
  return vector;
}

std::vector<double>
CordinateConverter::vectorScalarDivision(std::vector<double> vector, double a) {
  std::vector<double> output;
  for (int i = 0; i < vector.size(); i++) {
    output.push_back(vector[i] / a);
  }
  return output;
}

std::vector<double>
CordinateConverter::pointToVector(std::vector<double> start_point,
                                  std::vector<double> end_point) {

  return vectorSubtract(end_point, start_point);
}

std::vector<double> CordinateConverter::calcProjv(std::vector<double> vectorA,
                                                  std::vector<double> vectorB) {
  double projT = dotProudct(vectorA, vectorB) / dotProudct(vectorB, vectorB);
  vectorB[0] *= projT;
  vectorB[1] *= projT;
  return vectorB;
}

double CordinateConverter::dotProudct(std::vector<double> vectorA,
                                      std::vector<double> vectorB) {
  double dot = (vectorA[0] * vectorB[0]) + (vectorA[1] * vectorB[1]);
  return dot;
}

double CordinateConverter::crossProduct(std::vector<double> vectorA,
                                        std::vector<double> vectorB) {
  double cross = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0];
  return cross;
}

int CordinateConverter::getStartPathFromFrenet(double s, double d) {
  int idx_counter = 0;
  if (s > getpathLenth())
    s = std::fmod(s, (getpathLenth()));
  if (s < 0)
    s += getpathLenth();
  while (calcPathDistance(0, idx_counter + 1) <= s) {
    idx_counter++;
  }
  return idx_counter;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cordinate_converter");
  CordinateConverter c(node, true);
  std::vector<double> a = c.globalToFrenet(0.1134318, 5.2239407);
  std::vector<double> b = c.FrenetToGlobal(a[0], a[1]);
  RCLCPP_INFO(node->get_logger(), "entire path lenth: %.3f", c.getpathLenth());
  RCLCPP_INFO(node->get_logger(), "frenet frame : s %.3f, d %.3f", a[0], a[1]);
  RCLCPP_INFO(node->get_logger(), "decode : s %.3f, d %.5f", b[0], b[1]);
  rclcpp::spin(node);
}