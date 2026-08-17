#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Cloud::Ptr voxel_downsample(const Cloud::ConstPtr &input, float leaf_size)
{
  auto output = std::make_shared<Cloud>();
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_rotation(const Eigen::Matrix3f &rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double roll_from_rotation(const Eigen::Matrix3f &rotation)
{
  return std::atan2(rotation(2, 1), rotation(2, 2));
}

double pitch_from_rotation(const Eigen::Matrix3f &rotation)
{
  return std::asin(std::clamp(-static_cast<double>(rotation(2, 0)), -1.0, 1.0));
}

double alignment_fitness(
  const Cloud::ConstPtr &source, pcl::KdTreeFLANN<Point> &target_tree,
  const Eigen::Matrix4f &transform, double max_correspondence_distance)
{
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, transform);
  const float max_squared_distance = static_cast<float>(
    max_correspondence_distance * max_correspondence_distance);
  double squared_distance_sum = 0.0;
  std::size_t correspondence_count = 0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto &point : transformed) {
    if (target_tree.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
        squared_distances[0] <= max_squared_distance) {
      squared_distance_sum += squared_distances[0];
      ++correspondence_count;
    }
  }
  return correspondence_count == 0 ?
         std::numeric_limits<double>::infinity() :
         squared_distance_sum / static_cast<double>(correspondence_count);
}

}  // namespace

class FixedMapLocalizer : public rclcpp::Node
{
public:
  FixedMapLocalizer() : Node("fixed_map_localizer")
  {
    declare_parameter<std::string>("map_file_path", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("source_topic", "/cloud_registered");
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<double>("update_rate_hz", 1.0);
    declare_parameter<double>("tf_heartbeat_rate_hz", 5.0);
    declare_parameter<double>("max_tf_heartbeat_age_s", 2.5);
    declare_parameter<double>("map_voxel_size_m", 0.30);
    declare_parameter<double>("scan_voxel_size_m", 0.20);
    declare_parameter<double>("max_correspondence_distance_m", 2.0);
    declare_parameter<double>("max_fitness_score", 0.10);
    declare_parameter<int>("maximum_iterations", 40);
    declare_parameter<bool>("planar_mode", true);
    declare_parameter<double>("max_translation_step_m", 0.10);
    declare_parameter<double>("max_yaw_step_rad", 0.0872665);
    declare_parameter<double>("max_initial_translation_m", 1.0);
    declare_parameter<double>("max_initial_yaw_rad", 0.523599);
    declare_parameter<double>("initial_x_m", 0.0);
    declare_parameter<double>("initial_y_m", 0.0);
    declare_parameter<double>("initial_yaw_rad", 0.0);

    const auto map_path = get_parameter("map_file_path").as_string();
    if (map_path.empty()) {
      throw std::invalid_argument("map_file_path is required");
    }
    if (get_parameter("update_rate_hz").as_double() <= 0.0 ||
        get_parameter("tf_heartbeat_rate_hz").as_double() <= 0.0 ||
        get_parameter("max_tf_heartbeat_age_s").as_double() <= 0.0 ||
        get_parameter("map_voxel_size_m").as_double() <= 0.0 ||
        get_parameter("scan_voxel_size_m").as_double() <= 0.0 ||
        get_parameter("max_correspondence_distance_m").as_double() <= 0.0 ||
        get_parameter("max_fitness_score").as_double() <= 0.0 ||
        get_parameter("maximum_iterations").as_int() <= 0 ||
        get_parameter("max_translation_step_m").as_double() <= 0.0 ||
        get_parameter("max_yaw_step_rad").as_double() <= 0.0 ||
        get_parameter("max_initial_translation_m").as_double() <= 0.0 ||
        get_parameter("max_initial_yaw_rad").as_double() <= 0.0) {
      throw std::invalid_argument("localization rates, distances, gates, and iterations must be positive");
    }
    auto raw_map = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<Point>(map_path, *raw_map) != 0 || raw_map->empty()) {
      throw std::runtime_error("unable to load non-empty PCD: " + map_path);
    }
    map_ = voxel_downsample(raw_map, get_parameter("map_voxel_size_m").as_double());
    if (map_->empty()) {
      throw std::runtime_error("map voxel filter removed every point");
    }
    map_tree_.setInputCloud(map_);
    RCLCPP_INFO(get_logger(), "Loaded fixed map: %zu -> %zu points", raw_map->size(), map_->size());

    const double yaw = get_parameter("initial_yaw_rad").as_double();
    estimate_ = Eigen::Translation3f(
      static_cast<float>(get_parameter("initial_x_m").as_double()),
      static_cast<float>(get_parameter("initial_y_m").as_double()), 0.0F) *
      Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
    initial_estimate_ = estimate_;

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fixed_map_localization/odometry", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/fixed_map_localization/diagnostics", 10);
    if (get_parameter("publish_tf").as_bool()) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    heartbeat_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = cloud_callback_group_;
    const auto heartbeat_period = std::chrono::duration<double>(
      1.0 / get_parameter("tf_heartbeat_rate_hz").as_double());
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
      std::bind(&FixedMapLocalizer::publish_heartbeat, this), heartbeat_callback_group_);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("source_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&FixedMapLocalizer::cloud_callback, this, std::placeholders::_1),
      subscription_options);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    const double interval = 1.0 / get_parameter("update_rate_hz").as_double();
    if (last_update_.nanoseconds() != 0 && (now - last_update_).seconds() < interval) {
      return;
    }
    if (msg->header.frame_id != get_parameter("odom_frame").as_string()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Ignoring cloud frame '%s'; expected '%s'", msg->header.frame_id.c_str(),
        get_parameter("odom_frame").as_string().c_str());
      return;
    }

    auto raw_scan = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *raw_scan);
    if (raw_scan->empty()) {
      return;
    }
    last_update_ = now;
    auto scan = voxel_downsample(raw_scan, get_parameter("scan_voxel_size_m").as_double());
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(scan);
    icp.setInputTarget(map_);
    icp.setMaximumIterations(get_parameter("maximum_iterations").as_int());
    icp.setMaxCorrespondenceDistance(get_parameter("max_correspondence_distance_m").as_double());
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-4);
    Cloud aligned;
    Eigen::Isometry3f initial_guess;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      initial_guess = estimate_;
    }
    icp.align(aligned, initial_guess.matrix());
    const double raw_fitness = icp.getFitnessScore();
    if (!icp.hasConverged() || !std::isfinite(raw_fitness) ||
        raw_fitness > get_parameter("max_fitness_score").as_double()) {
      record_rejection(&raw_icp_rejections_, "raw_icp_rejected");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Fixed-map ICP rejected: converged=%s fitness=%.4f",
        icp.hasConverged() ? "true" : "false", raw_fitness);
      return;
    }

    Eigen::Isometry3f candidate(icp.getFinalTransformation());
    Eigen::Isometry3f reference;
    bool has_accepted_alignment;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      has_accepted_alignment = has_accepted_alignment_;
      reference = has_accepted_alignment ? estimate_ : initial_estimate_;
    }
    if (get_parameter("planar_mode").as_bool() && has_accepted_alignment) {
      const float roll = static_cast<float>(roll_from_rotation(reference.rotation()));
      const float pitch = static_cast<float>(pitch_from_rotation(reference.rotation()));
      const float yaw = static_cast<float>(yaw_from_rotation(candidate.rotation()));
      candidate.linear() = (
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
      candidate.translation().z() = reference.translation().z();
    }

    const double constrained_fitness = alignment_fitness(
      scan, map_tree_, candidate.matrix(),
      get_parameter("max_correspondence_distance_m").as_double());
    if (!std::isfinite(constrained_fitness) ||
        constrained_fitness > get_parameter("max_fitness_score").as_double()) {
      record_rejection(&constrained_fitness_rejections_, "constrained_fitness_rejected");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Fixed-map constrained candidate rejected: raw_fitness=%.4f constrained_fitness=%.4f",
        raw_fitness, constrained_fitness);
      return;
    }

    const Eigen::Vector2f translation_delta =
      candidate.translation().head<2>() - reference.translation().head<2>();
    const double translation_step = translation_delta.norm();
    const double yaw_step = std::abs(normalize_angle(
      yaw_from_rotation(candidate.rotation()) - yaw_from_rotation(reference.rotation())));
    if (has_accepted_alignment) {
        if (translation_step > get_parameter("max_translation_step_m").as_double() ||
            yaw_step > get_parameter("max_yaw_step_rad").as_double()) {
          record_rejection(&motion_gate_rejections_, "motion_gate_rejected");
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Fixed-map ICP motion gate rejected: translation_step=%.4f yaw_step=%.4f fitness=%.4f",
          translation_step, yaw_step, constrained_fitness);
        return;
      }
    } else if (translation_step > get_parameter("max_initial_translation_m").as_double() ||
               yaw_step > get_parameter("max_initial_yaw_rad").as_double()) {
      record_rejection(&initial_gate_rejections_, "initial_gate_rejected");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Fixed-map ICP initial gate rejected: translation=%.4f yaw=%.4f fitness=%.4f",
        translation_step, yaw_step, constrained_fitness);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      estimate_ = candidate;
      has_accepted_alignment_ = true;
      last_accepted_fitness_ = constrained_fitness;
      last_accepted_time_ = std::chrono::steady_clock::now();
      ++accepted_updates_;
      heartbeat_stale_ = false;
      // Serialize TF with the heartbeat so an older estimate cannot acquire a newer stamp.
      publish_estimate(msg->header.stamp, get_clock()->now(), candidate, constrained_fitness);
    }
    publish_diagnostics("accepted");
  }

  void publish_heartbeat()
  {
    bool awaiting_initial_alignment = false;
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      if (!has_accepted_alignment_) {
        awaiting_initial_alignment = true;
      } else {
        stale = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_accepted_time_).count() >
          get_parameter("max_tf_heartbeat_age_s").as_double();
        if (stale) {
          heartbeat_stale_ = true;
        } else {
          // Hold the lock through publication to preserve estimate/timestamp order.
          publish_transform(get_clock()->now(), estimate_);
          return;
        }
      }
    }
    if (awaiting_initial_alignment) {
      publish_diagnostics("awaiting_initial_alignment");
      return;
    }
    publish_diagnostics("heartbeat_stale");
  }

  void record_rejection(std::size_t *counter, const std::string &event)
  {
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      ++*counter;
    }
    publish_diagnostics(event);
  }

  void publish_diagnostics(const std::string &event)
  {
    std::size_t accepted_updates;
    std::size_t raw_icp_rejections;
    std::size_t constrained_fitness_rejections;
    std::size_t motion_gate_rejections;
    std::size_t initial_gate_rejections;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      accepted_updates = accepted_updates_;
      raw_icp_rejections = raw_icp_rejections_;
      constrained_fitness_rejections = constrained_fitness_rejections_;
      motion_gate_rejections = motion_gate_rejections_;
      initial_gate_rejections = initial_gate_rejections_;
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fixed_map_localization";
    status.hardware_id = "fixed_map_localizer";
    status.level = event == "accepted" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      (event == "heartbeat_stale" ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = event;
    const auto add_counter = [&status](const std::string &name, std::size_t value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = name;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
    add_counter("accepted_updates", accepted_updates);
    add_counter("raw_icp_rejections", raw_icp_rejections);
    add_counter("constrained_fitness_rejections", constrained_fitness_rejections);
    add_counter("motion_gate_rejections", motion_gate_rejections);
    add_counter("initial_gate_rejections", initial_gate_rejections);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(std::move(status));
    diagnostics_pub_->publish(diagnostics);
  }

  void publish_estimate(
    const builtin_interfaces::msg::Time &odometry_stamp,
    const builtin_interfaces::msg::Time &transform_stamp,
    const Eigen::Isometry3f &estimate, double fitness)
  {
    const Eigen::Quaternionf rotation(estimate.rotation());
    const auto &translation = estimate.translation();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = odometry_stamp;
    odometry.header.frame_id = get_parameter("map_frame").as_string();
    odometry.child_frame_id = get_parameter("odom_frame").as_string();
    odometry.pose.pose.position.x = translation.x();
    odometry.pose.pose.position.y = translation.y();
    odometry.pose.pose.position.z = translation.z();
    odometry.pose.pose.orientation.x = rotation.x();
    odometry.pose.pose.orientation.y = rotation.y();
    odometry.pose.pose.orientation.z = rotation.z();
    odometry.pose.pose.orientation.w = rotation.w();
    odometry.pose.covariance[0] = fitness;
    odometry.pose.covariance[7] = fitness;
    odometry.pose.covariance[35] = fitness;
    odom_pub_->publish(odometry);

    publish_transform(transform_stamp, estimate);
  }

  void publish_transform(
    const builtin_interfaces::msg::Time &stamp, const Eigen::Isometry3f &estimate)
  {
    if (!tf_broadcaster_) {
      return;
    }
    const Eigen::Quaternionf rotation(estimate.rotation());
    const auto &translation = estimate.translation();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = get_parameter("map_frame").as_string();
    transform.child_frame_id = get_parameter("odom_frame").as_string();
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    tf_broadcaster_->sendTransform(transform);
  }

  Cloud::Ptr map_;
  pcl::KdTreeFLANN<Point> map_tree_;
  Eigen::Isometry3f estimate_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f initial_estimate_{Eigen::Isometry3f::Identity()};
  std::mutex estimate_mutex_;
  bool has_accepted_alignment_{false};
  bool heartbeat_stale_{false};
  double last_accepted_fitness_{std::numeric_limits<double>::infinity()};
  std::chrono::steady_clock::time_point last_accepted_time_{};
  std::size_t accepted_updates_{0};
  std::size_t raw_icp_rejections_{0};
  std::size_t constrained_fitness_rejections_{0};
  std::size_t motion_gate_rejections_{0};
  std::size_t initial_gate_rejections_{0};
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_callback_group_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<FixedMapLocalizer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &error) {
    std::cerr << "Fixed-map localization startup failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
