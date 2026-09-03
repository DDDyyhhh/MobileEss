#include "timed_dynamic_obstacle_layer/timed_dynamic_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/time.hpp"

namespace timed_dynamic_obstacle_layer
{

void TimedDynamicObstacleLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/nav2/mid360_obstacles")));
  declareParameter("ttl_seconds", rclcpp::ParameterValue(0.8));
  declareParameter("obstacle_cell_size", rclcpp::ParameterValue(0.05));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.5));
  declareParameter("source_timeout", rclcpp::ParameterValue(0.6));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TimedDynamicObstacleLayer lost its lifecycle node");
  }
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic_);
  node->get_parameter(getFullName("ttl_seconds"), ttl_seconds_);
  node->get_parameter(getFullName("obstacle_cell_size"), cell_size_);
  node->get_parameter(getFullName("transform_tolerance"), transform_tolerance_seconds_);
  node->get_parameter(getFullName("source_timeout"), source_timeout_seconds_);
  if (ttl_seconds_ <= 0.0 || cell_size_ <= 0.0 ||
    transform_tolerance_seconds_ < 0.0 || source_timeout_seconds_ <= 0.0)
  {
    throw std::invalid_argument("Invalid TimedDynamicObstacleLayer parameters");
  }

  matchSize();
  setDefaultValue(nav2_costmap_2d::FREE_SPACE);
  resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
  current_ = true;

  subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&TimedDynamicObstacleLayer::cloudCallback, this, std::placeholders::_1));
}

void TimedDynamicObstacleLayer::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  if (!cloud || cloud->header.frame_id.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), cloud->header.frame_id,
      tf2_ros::fromMsg(cloud->header.stamp),
      tf2::durationFromSec(transform_tolerance_seconds_));
  } catch (const tf2::TransformException &) {
    return;
  }

  sensor_msgs::msg::PointCloud2 transformed;
  try {
    tf2::doTransform(*cloud, transformed, transform);
  } catch (const tf2::TransformException &) {
    return;
  }

  const double now_seconds = clock_->now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  // A successfully transformed cloud, including an empty mark cloud, proves
  // the sensing/TF path is current enough to safely advance dynamic expiry.
  last_valid_cloud_seconds_ = now_seconds;
  for (sensor_msgs::PointCloud2ConstIterator<float> x_it(transformed, "x"),
    y_it(transformed, "y"), z_it(transformed, "z");
    x_it != x_it.end(); ++x_it, ++y_it, ++z_it)
  {
    if (!std::isfinite(*x_it) || !std::isfinite(*y_it) || !std::isfinite(*z_it)) {
      continue;
    }
    const CellKey key{
      static_cast<int>(std::floor(*x_it / cell_size_)),
      static_cast<int>(std::floor(*y_it / cell_size_))};
    marks_[key] = Mark{*x_it, *y_it, now_seconds};
  }
}

bool TimedDynamicObstacleLayer::hasHealthyInput(double now_seconds) const
{
  return last_valid_cloud_seconds_ >= 0.0 &&
         now_seconds - last_valid_cloud_seconds_ <= source_timeout_seconds_;
}

void TimedDynamicObstacleLayer::purgeExpired(double now_seconds)
{
  if (!hasHealthyInput(now_seconds)) {
    return;
  }
  for (auto it = marks_.begin(); it != marks_.end();) {
    if (now_seconds - it->second.last_seen_seconds >= ttl_seconds_) {
      it = marks_.erase(it);
    } else {
      ++it;
    }
  }
}

void TimedDynamicObstacleLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (layered_costmap_->isRolling()) {
    updateOrigin(
      robot_x - getSizeInMetersX() / 2.0,
      robot_y - getSizeInMetersY() / 2.0);
  }
  purgeExpired(clock_->now().seconds());
  // Rebuild this transient layer over the complete rolling window each cycle.
  // This is bounded by the 8 m x 8 m local map and guarantees that expired
  // dynamic cells disappear without raytrace geometry. StaticLayer remains a
  // separate lower layer and is not modified by this reset.
  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
  *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
  resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
}

void TimedDynamicObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & entry : marks_) {
    unsigned int mx;
    unsigned int my;
    if (!worldToMap(entry.second.world_x, entry.second.world_y, mx, my)) {
      continue;
    }
    if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }
    setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void TimedDynamicObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  marks_.clear();
  resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(),
    nav2_costmap_2d::FREE_SPACE);
  current_ = true;
}

}  // namespace timed_dynamic_obstacle_layer

PLUGINLIB_EXPORT_CLASS(
  timed_dynamic_obstacle_layer::TimedDynamicObstacleLayer,
  nav2_costmap_2d::Layer)
