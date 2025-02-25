#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
//#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pgm_to_occupancy_grid_node.cpp"
//#include "pgm_to_occupancy_grid_node.cpp"
//#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "geometry_msgs/msg/pose.hpp"

class CordinateConverter {
public:
  CordinateConverter(rclcpp::Node::SharedPtr node, bool node_mode,
                     std::string config_file_path, const std::string path_type);
  ~CordinateConverter();

  std::vector<std::vector<double>> getGlobalPath();

  std::vector<double> globalToFrenet(double x, double y);
  std::vector<double> FrenetToGlobal(double s, double d);
  double getpathLenth();

private:
  int getClosestsIndex(double x, double y);

  int getStartPathFromFrenet(double s, double d);

  void readPath(std::string path_file_path);

  void readPath(nav_msgs::msg::Path path_topic);

  double calcPathToPathHeading(int idx);

  double calcPathToPathDistance(int idx);

  double calcPathToPathRechingTime(int idx);

  double calcDistance(double x, double y, double x1, double y1);

  double calcPathDistance(int idx_start, int idx_end);

  std::pair<double, double> calcDS(int idx, int next_idx, double x, double y);

  std::vector<double> calcProjv(std::vector<double> vectorA,
                                std::vector<double> vectorB);

  double crossProduct(std::vector<double> vectorA, std::vector<double> vectorB);

  double dotProudct(std::vector<double> vectorA, std::vector<double> vectorB);

  std::vector<double> toNormal2DVector(std::vector<double> vector);
  std::vector<double> pointToVector(std::vector<double> start_point,
                                    std::vector<double> end_point);
  std::vector<double> vectorScalarDivision(std::vector<double> vector,
                                           double a);
  std::vector<double> vectorScalarMultiple(std::vector<double> vector,
                                           double a);
  std::vector<double> vectorAdd(std::vector<double> vectorA,
                                std::vector<double> vectorB);

  void path_publisher();

  void path_msg_generator();

  void pathCallback(nav_msgs::msg::Path msg);

  bool node_mode_;

  bool path_recives;

  std::vector<std::vector<double>> global_path_;
  std::vector<double> global_path_heading_;
  std::vector<double> global_path_distance_;
  std::vector<double> global_path_reaching_time_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path::SharedPtr global_path_msg_;
  OccupancyGridNode *map_node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};
