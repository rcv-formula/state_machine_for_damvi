#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm> // std::max 사용
#include <bits/algorithmfwd.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class OccupancyGridNode {
public:
  OccupancyGridNode(rclcpp::Node::SharedPtr node, std::string pgm_path,
                    std::string yaml_path) {
    node_ = node;
    pgm_file_ = pgm_path;
    yaml_file_ = yaml_path;
    inflation_radius_ = 0.30;

    if (!pgm_file_.empty() && !yaml_file_.empty()) {
      loadMap();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Missing PGM or YAML file");
    }
    map_publisher_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>("global_map", 20);
    publish_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&OccupancyGridNode::publishMap, this));
  }

  uint8_t getPixel(int x, int y) { return map_[y][x]; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string pgm_file_;
  std::string yaml_file_;
  double inflation_radius_;
  nav_msgs::msg::OccupancyGrid map_data_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  double resolution;
  double origin[3];
  bool negate;
  double occupied_thresh;
  double free_thresh;
  std::vector<std::vector<int8_t>> map_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  void loadMap() {
    RCLCPP_INFO(node_->get_logger(), "loading a map: %s and %s",
                pgm_file_.c_str(), yaml_file_.c_str());

    // YAML 파일 로딩
    std::ifstream yaml_file_stream(yaml_file_);
    if (!yaml_file_stream.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "failed to open YAML: %s",
                   yaml_file_.c_str());
      return;
    }

    std::string line;
    while (std::getline(yaml_file_stream, line)) {
      std::istringstream ss(line);
      std::string key;
      ss >> key;
      if (key == "resolution:") {
        ss >> resolution;
      } else if (key == "origin:") {
        char delimiter;
        ss >> delimiter >> origin[0] >> delimiter >> origin[1] >> delimiter >>
            origin[2];
      } else if (key == "negate:") {
        ss >> negate;
      } else if (key == "occupied_thresh:") {
        ss >> occupied_thresh;
      } else if (key == "free_thresh:") {
        ss >> free_thresh;
      }
    }
    RCLCPP_INFO(node_->get_logger(),
                "yaml load compleate, resolution : %.3f, origin : (%.3f, "
                "%.3f), threash : %.3f, %.3f, negate : %d",
                resolution, origin[0], origin[1], occupied_thresh, free_thresh,
                negate);

    yaml_file_stream.close();

    // PGM 파일 로딩
    std::ifstream pgm_file_stream(pgm_file_, std::ios::binary);
    if (!pgm_file_stream.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open PGM file: %s",
                   pgm_file_.c_str());
      return;
    }

    std::string format;
    int width, height, max_value;
    pgm_file_stream >> format >> width >> height >> max_value;
    pgm_file_stream.ignore(1); // newline 무시

    std::vector<uint8_t> pgm_data(width * height);
    pgm_file_stream.read(reinterpret_cast<char *>(pgm_data.data()),
                         pgm_data.size());
    pgm_file_stream.close();

    // PGM 데이터를 OccupancyGrid 포맷으로 변환
    // -1: Unknown, 0: Free, 100: Occnegateupied
    std::vector<int8_t> occupancy_data(width * height, -1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int src_index = y * width + x;
        int dest_index = (height - 1 - y) * width + x; // 수직 반전

        double pixel_value =
            static_cast<double>(pgm_data[src_index]) / max_value;
        if (!negate) {
          pixel_value = 1.0 - pixel_value;
        }

        if (pixel_value > occupied_thresh) {
          occupancy_data[dest_index] = 100; // 장애물

        } else if (pixel_value < free_thresh) {
          occupancy_data[dest_index] = 0; // Free

        } else {
          occupancy_data[dest_index] = 100; // Unknown
        }
      }
    }

    // 인플레이션 적용: 장애물과 인접한 free cell에 대해 비용 부여
    inflateMap(occupancy_data, width, height, inflation_radius_, resolution);

    // OccupancyGrid 메시지 구성
    map_data_.info.resolution = resolution;
    map_data_.info.width = width;
    map_data_.info.height = height;
    map_data_.info.origin.position.x = origin[0];
    map_data_.info.origin.position.y = origin[1];
    map_data_.info.origin.position.z = 0.0;
    map_data_.info.origin.orientation.w = 1.0; // 회전 없음
    map_data_.info.origin.orientation.z = 0.0;
    map_data_.data = occupancy_data;

    RCLCPP_INFO(node_->get_logger(),
                "Map loading completed: %d x %d, resolution: %.3f", width,
                height, resolution);
  }

  // 인플레이션 함수: 장애물(100) 주변의 free 셀(0)에 대해, 거리에 따른
  // 비용(0~99)을 할당합니다.
  void inflateMap(std::vector<int8_t> &occupancy_data, int width, int height,
                  double inflation_radius, double resolution) {
    int inflation_radius_cells = std::ceil(inflation_radius / resolution);
    // 원본 데이터를 보존하기 위해 복사본 생성
    std::vector<int8_t> original = occupancy_data;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        if (original[idx] == 100) { // 장애물 셀인 경우
          // 장애물 주변의 셀들을 확인
          for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells;
               ++dy) {
            for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells;
                 ++dx) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
                int n_idx = ny * width + nx;
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance <= inflation_radius_cells) {
                  // 선형 감쇠를 이용하여 비용 계산 (장애물과 인접하면 비용이
                  // 높음)
                  double factor =
                      99.0 / static_cast<double>(inflation_radius_cells);
                  int cost = static_cast<int>(
                      std::round((inflation_radius_cells - distance) * factor));
                  cost = std::clamp(cost, 0, 99); // 0~99 범위로 제한
                  // 원래 free(0)인 셀에 대해서만 인플레이션 적용 (unknown은
                  // 그대로 유지)
                  if (original[n_idx] == 0) {
                    occupancy_data[n_idx] = std::max(occupancy_data[n_idx],
                                                     static_cast<int8_t>(cost));
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  void vector_converter(std::vector<int8_t> occupancy_data, int wide,
                        int height) {
    for (int y = 0; y < height; y++) {
      std::vector<int8_t> temp;
      for (int x = 0; x < wide; x++) {
        temp.push_back(occupancy_data[y * wide + x]);
      }
      map_.push_back(temp);
    }
  }

  void publishMap() {
    map_data_.header.stamp = node_->now();
    map_data_.header.frame_id = "map";
    map_publisher_->publish(map_data_);
  }
};

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<OccupancyGridNode>());
//     rclcpp::shutdown();
//     return 0;
// }
