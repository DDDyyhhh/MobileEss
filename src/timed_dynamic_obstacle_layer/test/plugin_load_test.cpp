#include <memory>

#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_loader.hpp"

int main()
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance(
    "timed_dynamic_obstacle_layer::TimedDynamicObstacleLayer");
  return layer ? 0 : 1;
}
