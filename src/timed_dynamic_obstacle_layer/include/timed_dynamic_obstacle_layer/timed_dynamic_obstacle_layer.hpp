#ifndef TIMED_DYNAMIC_OBSTACLE_LAYER__TIMED_DYNAMIC_OBSTACLE_LAYER_HPP_
#define TIMED_DYNAMIC_OBSTACLE_LAYER__TIMED_DYNAMIC_OBSTACLE_LAYER_HPP_

#include <map>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace timed_dynamic_obstacle_layer
{

class TimedDynamicObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
    int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  struct CellKey
  {
    int x;
    int y;

    bool operator<(const CellKey & other) const
    {
      return x == other.x ? y < other.y : x < other.x;
    }
  };

  struct Mark
  {
    double world_x;
    double world_y;
    double last_seen_seconds;
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void purgeExpired(double now_seconds);
  bool hasHealthyInput(double now_seconds) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::map<CellKey, Mark> marks_;
  std::mutex mutex_;
  std::string topic_;
  double ttl_seconds_ {0.8};
  double cell_size_ {0.1};
  double transform_tolerance_seconds_ {0.5};
  double source_timeout_seconds_ {0.6};
  double last_valid_cloud_seconds_ {-1.0};
};

}  // namespace timed_dynamic_obstacle_layer

#endif  // TIMED_DYNAMIC_OBSTACLE_LAYER__TIMED_DYNAMIC_OBSTACLE_LAYER_HPP_
