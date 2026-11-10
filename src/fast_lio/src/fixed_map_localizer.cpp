#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <atomic>
#include <cstdint>
#include <future>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
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
    declare_parameter<bool>("startup_search_enabled", false);
    declare_parameter<double>("startup_search_radius_m", 2.0);
    declare_parameter<double>("startup_search_step_m", 1.0);
    declare_parameter<int>("startup_yaw_samples", 24);
    declare_parameter<int>("startup_refine_candidates", 8);
    declare_parameter<double>("startup_min_overlap", 0.65);
    declare_parameter<double>("startup_ambiguity_margin", 0.01);
    declare_parameter<int>("startup_confirmation_count", 3);
    declare_parameter<std::string>("startup_search_mode", "near_hint");
    declare_parameter<double>("startup_search_timeout_s", 120.0);
    declare_parameter<double>("maximum_scan_age_s", 1.0);
    declare_parameter<bool>("require_stationary_startup", false);
    full_map_mode_ = get_parameter("startup_search_mode").as_string() == "full_map";
    if (get_parameter("startup_search_timeout_s").as_double() <= 0.0 ||
        get_parameter("maximum_scan_age_s").as_double() <= 0.0) {
      throw std::invalid_argument("search timeout and maximum scan age must be positive");
    }
    if (!full_map_mode_ && get_parameter("startup_search_mode").as_string() != "near_hint") {
      throw std::invalid_argument("startup_search_mode must be near_hint or full_map");
    }

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
    if (get_parameter("startup_search_radius_m").as_double() <= 0.0 ||
        get_parameter("startup_search_step_m").as_double() <= 0.0 ||
        get_parameter("startup_yaw_samples").as_int() < 4 ||
        get_parameter("startup_refine_candidates").as_int() < 2 ||
        get_parameter("startup_confirmation_count").as_int() < 2 ||
        get_parameter("startup_min_overlap").as_double() <= 0.0 ||
        get_parameter("startup_min_overlap").as_double() > 1.0 ||
        get_parameter("startup_ambiguity_margin").as_double() <= 0.0) {
      throw std::invalid_argument("invalid startup search parameters");
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
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg->info.resolution <= 0 || msg->data.size() != msg->info.width * msg->info.height) return;
        std::lock_guard<std::mutex> lock(grid_mutex_);
        navigation_grid_ = *msg;
      }, subscription_options);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", rclcpp::QoS(5),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        const double age=(get_clock()->now()-rclcpp::Time(msg->header.stamp)).seconds();
        if (age < -.05 || age > .30) return;
        const auto &p = msg->pose.pose;
        const Eigen::Quaternionf q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
        if (!std::isfinite(q.norm()) || q.norm() < .9F) return;
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_pose_ = Eigen::Translation3f(p.position.x,p.position.y,p.position.z) * q.normalized();
        odom_time_ = std::chrono::steady_clock::now();
        if (search_active_.load() &&
            ((odom_pose_.translation()-search_odom_.translation()).head<2>().norm() > .03 ||
             std::abs(normalize_angle(yaw_from_rotation(odom_pose_.rotation())-
                                     yaw_from_rotation(search_odom_.rotation()))) > .05236)) {
          std::lock_guard<std::mutex> generation_lock(generation_mutex_);
          if (search_active_.exchange(false)) {
            ++generation_;
            publish_diagnostics("startup_vehicle_moved");
          }
        }
      }, subscription_options);
    stationary_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/mower_console/stationary", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> generation_lock(generation_mutex_);
        stationary_.store(msg->data);
        stationary_time_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count());
        if (!msg->data && search_active_.exchange(false)) ++generation_;
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/fixed_map_localizer/relocalize",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = stationary_ready();
        response->message = response->success ? "global search requested" : "fresh API0 stationary evidence required";
        if (response->success) reset_localization(false, std::nullopt);
      }, rmw_qos_profile_services_default, cloud_callback_group_);
    hint_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (!stationary_ready() || msg->header.frame_id != get_parameter("map_frame").as_string()) return;
        const auto &p = msg->pose.pose;
        Eigen::Quaternionf q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
            !std::isfinite(q.norm()) || q.norm() < .9F) return;
        Eigen::Isometry3f odom;
        {std::lock_guard<std::mutex> lock(odom_mutex_); odom = odom_pose_;}
        Eigen::Isometry3f hint = Eigen::Translation3f(p.position.x,p.position.y,0.F) * q.normalized();
        reset_localization(true, hint * odom.inverse());
      }, subscription_options);
  }

  ~FixedMapLocalizer() override
  {
    ++generation_;
    search_active_.store(false);
    if (search_future_.valid()) search_future_.wait();
  }

private:
  bool stationary_ready() const
  {
    const auto age = std::chrono::steady_clock::now().time_since_epoch().count() - stationary_time_ns_.load();
    return stationary_.load() && age >= 0 &&
      static_cast<double>(age) * std::chrono::steady_clock::period::num /
      std::chrono::steady_clock::period::den < .30;
  }

  void reset_localization(bool hint, std::optional<Eigen::Isometry3f> pose)
  {
    std::lock_guard<std::mutex> generation_lock(generation_mutex_);
    ++generation_;
    search_active_.store(false);
    hint_mode_.store(hint);
    {std::lock_guard<std::mutex> lock(estimate_mutex_);
      has_accepted_alignment_ = false;
      startup_seed_ready_ = false;
      startup_confirmations_ = 0;
      startup_phase_.clear();
      if (pose) initial_estimate_ = *pose;
    }
    search_started_ = {};
    search_timed_out_ = false;
    publish_diagnostics("relocalization_requested");
  }

  bool search_cancelled(std::uint64_t generation,
                        std::chrono::steady_clock::time_point deadline) const
  {
    return generation != generation_.load() || !rclcpp::ok() ||
      std::chrono::steady_clock::now() > deadline ||
      (get_parameter("require_stationary_startup").as_bool() && !stationary_ready());
  }

  bool valid_map_base(const Eigen::Isometry3f &candidate, const Eigen::Isometry3f &odom,
                      const nav_msgs::msg::OccupancyGrid &grid) const
  {
    if (grid.data.empty()) return false;
    const Eigen::Vector3f xy = (candidate * odom).translation();
    const auto &o = grid.info.origin;
    const double a = std::atan2(2*(o.orientation.w*o.orientation.z+o.orientation.x*o.orientation.y),
                               1-2*(o.orientation.y*o.orientation.y+o.orientation.z*o.orientation.z));
    const double dx=xy.x()-o.position.x, dy=xy.y()-o.position.y;
    const int x=std::floor((std::cos(a)*dx+std::sin(a)*dy)/grid.info.resolution);
    const int y=std::floor((-std::sin(a)*dx+std::cos(a)*dy)/grid.info.resolution);
    return x>=0 && y>=0 && x<static_cast<int>(grid.info.width) &&
      y<static_cast<int>(grid.info.height) && grid.data[y*grid.info.width+x] == 0;
  }
  struct StartupCandidate
  {
    Eigen::Isometry3f pose;
    double score;
  };

  // Structural returns carry yaw/position information; the large flat floor
  // must not make distinct warehouse poses look equally reliable.
  std::pair<double, double> startup_score(
    const Cloud::ConstPtr &scan, const Eigen::Matrix4f &pose, double limit)
  {
    Cloud transformed;
    pcl::transformPointCloud(*scan, transformed, pose);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    double sum = 0.0;
    std::size_t matched = 0;
    for (const auto &point : transformed) {
      if (map_tree_.nearestKSearch(point, 1, indices, distances) == 1 &&
          distances[0] <= limit * limit) {
        sum += distances[0];
        ++matched;
      }
    }
    return {matched ? sum / matched : std::numeric_limits<double>::infinity(),
      transformed.empty() ? 0.0 : static_cast<double>(matched) / transformed.size()};
  }

  std::optional<Eigen::Isometry3f> search_startup(const Cloud::ConstPtr &scan, std::uint64_t generation,
    std::chrono::steady_clock::time_point deadline)
  {
    if (generation != generation_.load()) return std::nullopt;
    const bool full = full_map_mode_ && !hint_mode_.load();
    nav_msgs::msg::OccupancyGrid grid;
    Eigen::Isometry3f odom, initial;
    {std::lock_guard<std::mutex> lock(grid_mutex_); grid = navigation_grid_;}
    {std::lock_guard<std::mutex> lock(odom_mutex_); odom = odom_pose_;}
    {std::lock_guard<std::mutex> lock(estimate_mutex_); initial = initial_estimate_;}
    auto structure = std::make_shared<Cloud>();
    for (const auto &point : *scan) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          point.z >= -0.25F && point.z <= 3.0F) {
        structure->push_back(point);
      }
    }
    structure = voxel_downsample(structure, 0.6F);
    if (structure->size() < 30) {
      publish_diagnostics("startup_insufficient_structure");
      return std::nullopt;
    }
    // Bound coarse scoring work independently of the raw scan density.
    auto sampled = std::make_shared<Cloud>();
    const std::size_t stride = std::max<std::size_t>(1, (structure->size() + 399) / 400);
    for (std::size_t i = 0; i < structure->size(); i += stride) {
      sampled->push_back((*structure)[i]);
    }
    const double radius = get_parameter("startup_search_radius_m").as_double();
    const double step = get_parameter("startup_search_step_m").as_double();
    const int count = static_cast<int>(std::ceil(radius / step));
    const int yaw_samples = get_parameter("startup_yaw_samples").as_int();
    const double min_overlap = get_parameter("startup_min_overlap").as_double();
    std::vector<StartupCandidate> seeds;
    const int nx = full ? static_cast<int>(std::ceil(grid.info.width*grid.info.resolution/step)) : count;
    const int ny = full ? static_cast<int>(std::ceil(grid.info.height*grid.info.resolution/step)) : count;
    for (int ix = full ? 0 : -count; ix <= nx; ++ix) {
      for (int iy = full ? 0 : -count; iy <= ny; ++iy) {
        if (search_cancelled(generation,deadline)) return std::nullopt;
        const double dx = ix * step;
        const double dy = iy * step;
        if (!full && std::hypot(dx, dy) > radius + 1e-6) {
          continue;
        }
        for (int i = 0; i < yaw_samples; ++i) {
          const float yaw = static_cast<float>(2.0 * M_PI * i / yaw_samples);
          Eigen::Isometry3f pose = Eigen::Translation3f(
            initial.translation().x() + dx,
            initial.translation().y() + dy, 0.0F) *
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
          if (full) {
            const auto &o = grid.info.origin;
            const double a=std::atan2(2*o.orientation.w*o.orientation.z,1-2*o.orientation.z*o.orientation.z);
            Eigen::Isometry3f map_base = Eigen::Translation3f(
              o.position.x+std::cos(a)*dx-std::sin(a)*dy,
              o.position.y+std::sin(a)*dx+std::cos(a)*dy,0.F) *
              Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ());
            pose = map_base * odom.inverse();
            if (!valid_map_base(pose,odom,grid)) continue;
          }
          const auto score = startup_score(sampled, pose.matrix(), 1.0);
          if (score.second >= min_overlap) {
            seeds.push_back({pose, score.first});
          }
        }
      }
    }
    std::sort(seeds.begin(), seeds.end(), [](const auto &a, const auto &b) {
      return a.score < b.score;
    });
    if (full) {
      std::vector<StartupCandidate> independent;
      for (const auto &seed : seeds) {
        const bool duplicate = std::any_of(independent.begin(),independent.end(),[&](const StartupCandidate &other) {
          return (seed.pose.translation()-other.pose.translation()).head<2>().norm() < .75 &&
            std::abs(normalize_angle(yaw_from_rotation(seed.pose.rotation())-
                                    yaw_from_rotation(other.pose.rotation()))) < .35;
        });
        if (!duplicate) independent.push_back(seed);
        if (independent.size() > static_cast<std::size_t>(get_parameter("startup_refine_candidates").as_int())) break;
      }
      if (independent.size() > static_cast<std::size_t>(get_parameter("startup_refine_candidates").as_int()) &&
          independent.back().score-independent.front().score < get_parameter("startup_ambiguity_margin").as_double()) {
        publish_diagnostics("startup_candidates_ambiguous");
        return std::nullopt;
      }
      seeds = std::move(independent);
    }
    std::vector<StartupCandidate> solutions;
    const std::size_t refine_count = std::min<std::size_t>(seeds.size(),
      get_parameter("startup_refine_candidates").as_int());
    for (std::size_t i = 0; i < refine_count; ++i) {
      pcl::IterativeClosestPoint<Point, Point> coarse_icp;
      coarse_icp.setInputSource(structure);
      coarse_icp.setInputTarget(map_);
      coarse_icp.setMaximumIterations(30);
      coarse_icp.setMaxCorrespondenceDistance(1.0);
      coarse_icp.setTransformationEpsilon(1e-4);
      Cloud aligned;
      coarse_icp.align(aligned, seeds[i].pose.matrix());
      if (search_cancelled(generation,deadline)) return std::nullopt;
      if (!coarse_icp.hasConverged()) {
        continue;
      }
      const Eigen::Isometry3f pose(coarse_icp.getFinalTransformation());
      const auto score = startup_score(structure, pose.matrix(), 0.5);
      if (score.second < min_overlap || !std::isfinite(score.first) ||
          score.first > get_parameter("max_fitness_score").as_double() ||
          (!full && (pose.translation().head<2>() - initial.translation().head<2>()).norm() > radius) ||
          (full && !valid_map_base(pose,odom,grid))) {
        continue;
      }
      solutions.push_back({pose, score.first});
    }
    std::sort(solutions.begin(), solutions.end(), [](const auto &a, const auto &b) {
      return a.score < b.score;
    });
    if (solutions.empty()) {
      publish_diagnostics("startup_search_rejected");
      return std::nullopt;
    }
    const auto &best = solutions.front();
    for (std::size_t i = 1; i < solutions.size(); ++i) {
      const auto &other = solutions[i];
      const double distance = (best.pose.translation().head<2>() - other.pose.translation().head<2>()).norm();
      const double yaw = std::abs(normalize_angle(yaw_from_rotation(best.pose.rotation()) -
        yaw_from_rotation(other.pose.rotation())));
      if ((distance > 0.5 || yaw > 0.261799) &&
          other.score - best.score < get_parameter("startup_ambiguity_margin").as_double()) {
        publish_diagnostics("startup_ambiguous");
        return std::nullopt;
      }
    }
    return best.pose;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    const double source_age = (now-rclcpp::Time(msg->header.stamp)).seconds();
    if (source_age < -.05 || source_age > get_parameter("maximum_scan_age_s").as_double()) {
      publish_diagnostics("stale_source_cloud"); return;
    }
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
    const bool startup_enabled = get_parameter("startup_search_enabled").as_bool();
    if (startup_enabled && !has_accepted_alignment_) {
      if (startup_seed_ready_ && search_generation_ != generation_.load()) {
        startup_seed_ready_=false;startup_confirmations_=0;
      }
      if (search_timed_out_) {publish_diagnostics("startup_timeout");return;}
      if (get_parameter("require_stationary_startup").as_bool()) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (std::chrono::duration<double>(std::chrono::steady_clock::now()-odom_time_).count() > .30) {
          ++generation_;startup_seed_ready_=false;startup_confirmations_=0;
          publish_diagnostics("startup_waiting_odometry");return;
        }
      }
      if (get_parameter("require_stationary_startup").as_bool() && !stationary_ready()) {
        ++generation_; startup_seed_ready_=false;startup_confirmations_=0;
        publish_diagnostics("startup_waiting_stationary");return;
      }
      if (search_started_ == std::chrono::steady_clock::time_point{}) search_started_ = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-search_started_).count() >
          get_parameter("startup_search_timeout_s").as_double()) {
        ++generation_;search_timed_out_=true;publish_diagnostics("startup_timeout");return;
      }
      if (!startup_seed_ready_) {
        if (!search_future_.valid()) {
          if (full_map_mode_ && !hint_mode_.load()) {
            std::lock_guard<std::mutex> lock(grid_mutex_);
            if (navigation_grid_.data.empty()) {publish_diagnostics("startup_waiting_map");return;}
          }
          search_generation_ = generation_.load();
          {std::lock_guard<std::mutex> lock(odom_mutex_); search_odom_ = odom_pose_;}
          search_active_.store(get_parameter("require_stationary_startup").as_bool());
          const auto expected_generation = search_generation_;
          const auto deadline = search_started_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(get_parameter("startup_search_timeout_s").as_double()));
          search_future_ = std::async(std::launch::async,[this,scan,expected_generation,deadline] {
            return search_startup(scan,expected_generation,deadline);});
          publish_diagnostics("startup_searching");return;
        }
        if (search_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
        const auto seed=search_future_.get();
        if (!seed || search_generation_ != generation_.load()) return;
        if (get_parameter("require_stationary_startup").as_bool()) {
          std::lock_guard<std::mutex> lock(odom_mutex_);
          if ((odom_pose_.translation()-search_odom_.translation()).head<2>().norm() > .03 ||
              std::abs(normalize_angle(yaw_from_rotation(odom_pose_.rotation())-
                                      yaw_from_rotation(search_odom_.rotation()))) > .05236) {
            ++generation_;publish_diagnostics("startup_vehicle_moved");return;
          }
        }
        startup_pending_=*seed;
        startup_seed_ready_ = true;
        publish_diagnostics("startup_confirming");return;
      }
      if (startup_seed_ready_ && get_parameter("require_stationary_startup").as_bool()) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if ((odom_pose_.translation()-search_odom_.translation()).head<2>().norm() > .03 ||
            std::abs(normalize_angle(yaw_from_rotation(odom_pose_.rotation())-
                                    yaw_from_rotation(search_odom_.rotation()))) > .05236) {
          ++generation_;startup_seed_ready_=false;startup_confirmations_=0;
          publish_diagnostics("startup_vehicle_moved");return;
        }
      }
      initial_guess = startup_pending_;
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
    } else if ((!full_map_mode_ || hint_mode_.load()) &&
               (translation_step > get_parameter("max_initial_translation_m").as_double() ||
               yaw_step > get_parameter("max_initial_yaw_rad").as_double())) {
      record_rejection(&initial_gate_rejections_, "initial_gate_rejected");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Fixed-map ICP initial gate rejected: translation=%.4f yaw=%.4f fitness=%.4f",
        translation_step, yaw_step, constrained_fitness);
      return;
    }

    if (startup_enabled && !has_accepted_alignment) {
      if (full_map_mode_ && !hint_mode_.load()) {
        std::scoped_lock lock(grid_mutex_,odom_mutex_);
        if (!valid_map_base(candidate,odom_pose_,navigation_grid_)) {
          startup_seed_ready_=false;startup_confirmations_=0;
          publish_diagnostics("startup_outside_free_map");return;
        }
      }
      const double difference = (candidate.translation().head<2>() - startup_pending_.translation().head<2>()).norm();
      const double angle = std::abs(normalize_angle(yaw_from_rotation(candidate.rotation()) -
        yaw_from_rotation(startup_pending_.rotation())));
      if (difference > get_parameter("max_translation_step_m").as_double() ||
          angle > get_parameter("max_yaw_step_rad").as_double()) {
        startup_seed_ready_ = false;
        startup_confirmations_ = 0;
        publish_diagnostics("startup_unstable");
        return;
      }
      startup_pending_ = candidate;
      ++startup_confirmations_;
      if (startup_confirmations_ < get_parameter("startup_confirmation_count").as_int()) {
        publish_diagnostics("startup_confirming");
        return;
      }
    }

    if ((get_clock()->now()-rclcpp::Time(msg->header.stamp)).seconds() >
        get_parameter("maximum_scan_age_s").as_double()) {
      publish_diagnostics("stale_computed_alignment");return;
    }
    std::lock_guard<std::mutex> generation_lock(generation_mutex_);
    if (startup_enabled && !has_accepted_alignment &&
        (search_generation_ != generation_.load() ||
         (get_parameter("require_stationary_startup").as_bool() && !stationary_ready()))) {
      startup_seed_ready_=false;startup_confirmations_=0;
      publish_diagnostics("startup_invalidated");return;
    }
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      estimate_ = candidate;
      has_accepted_alignment_ = true;
      search_active_.store(false);
      last_accepted_fitness_ = constrained_fitness;
      last_accepted_time_ = std::chrono::steady_clock::now() -
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(std::max(0.0,(get_clock()->now()-rclcpp::Time(msg->header.stamp)).seconds())));
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
      if (!has_accepted_alignment_) {
        startup_seed_ready_ = false;
        startup_confirmations_ = 0;
      }
    }
    publish_diagnostics(event);
  }

  void publish_diagnostics(const std::string &event)
  {
    std::string displayed_event = event;
    std::size_t accepted_updates;
    std::size_t raw_icp_rejections;
    std::size_t constrained_fitness_rejections;
    std::size_t motion_gate_rejections;
    std::size_t initial_gate_rejections;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      if (!has_accepted_alignment_) {
        if (event == "awaiting_initial_alignment" && !startup_phase_.empty()) {
          displayed_event = startup_phase_;
        } else if (event.rfind("startup_",0) == 0 || event == "relocalization_requested") {
          startup_phase_ = event;
        }
      }
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
    status.message = displayed_event;
    const auto add_counter = [&status](const std::string &name, std::size_t value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = name;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
    add_counter("accepted_updates", accepted_updates);
    add_counter("generation", generation_.load());
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
  bool full_map_mode_{false};
  std::string startup_phase_;
  std::atomic<bool> hint_mode_{false}, stationary_{false};
  std::atomic<std::int64_t> stationary_time_ns_{0};
  std::atomic<std::uint64_t> generation_{0};
  std::atomic<bool> search_active_{false};
  std::mutex generation_mutex_;
  std::uint64_t search_generation_{0};
  std::future<std::optional<Eigen::Isometry3f>> search_future_;
  std::chrono::steady_clock::time_point search_started_{}, odom_time_{};
  bool search_timed_out_{false};
  Eigen::Isometry3f odom_pose_{Eigen::Isometry3f::Identity()},search_odom_{Eigen::Isometry3f::Identity()};
  nav_msgs::msg::OccupancyGrid navigation_grid_;
  std::mutex grid_mutex_,odom_mutex_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stationary_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr hint_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  pcl::KdTreeFLANN<Point> map_tree_;
  Eigen::Isometry3f estimate_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f initial_estimate_{Eigen::Isometry3f::Identity()};
  std::mutex estimate_mutex_;
  bool has_accepted_alignment_{false};
  bool startup_seed_ready_{false};
  int startup_confirmations_{0};
  Eigen::Isometry3f startup_pending_{Eigen::Isometry3f::Identity()};
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
