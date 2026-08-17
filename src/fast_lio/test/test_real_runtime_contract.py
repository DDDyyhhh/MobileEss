from pathlib import Path
import ast


REPO_ROOT = Path(__file__).resolve().parents[3]
FAST_LIO_ROOT = REPO_ROOT / "src" / "fast_lio"


def test_real_launch_requires_an_external_calibrated_config():
    launch = (FAST_LIO_ROOT / "launch" / "fast_lio_real.launch.py").read_text()

    ast.parse(launch)
    assert "FAST_LIO_REAL_CONFIG" in launch
    assert "/etc/mower/fast_lio/mid360_real.yaml" in launch
    assert '"use_sim_time": LaunchConfiguration("use_sim_time")' in launch
    assert "publish.tf_en" in launch
    assert "mid360.yaml" not in launch


def test_fast_lio_real_path_uses_native_custom_msg_and_sensor_qos():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()
    preprocess = (FAST_LIO_ROOT / "src" / "preprocess.cpp").read_text()
    header = (FAST_LIO_ROOT / "src" / "preprocess.h").read_text()

    assert "livox_ros_driver2/msg/custom_msg.hpp" in source
    assert "create_subscription<livox_ros_driver2::msg::CustomMsg>" in source
    assert "rclcpp::SensorDataQoS()" in source
    assert "publish.tf_en" in source
    assert "if (tf_pub_en)" in source
    assert "tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);" in source
    assert "if (tf_pub_en && tf_br)" in source
    assert "publish_odometry(pubOdomAftMapped_, tf_broadcaster_);" in source
    assert "msg->point_num != msg->points.size()" in preprocess
    assert "offset_time / float(1000000)" in preprocess
    assert "livox_ros_driver2/msg/custom_msg.hpp" in header


def test_real_odometry_interface_is_parameterized_for_ekf_input():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()
    launch = (FAST_LIO_ROOT / "launch" / "fast_lio_real.launch.py").read_text()
    config = (FAST_LIO_ROOT / "config" / "mid360_real.yaml").read_text()

    assert 'declare_parameter<string>("common.odom_frame"' in source
    assert 'declare_parameter<string>("common.body_frame"' in source
    assert 'declare_parameter<string>("common.odometry_topic"' in source
    assert "odomAftMapped.header.frame_id = odom_frame;" in source
    assert "odomAftMapped.child_frame_id = body_frame;" in source
    assert "create_publisher<nav_msgs::msg::Odometry>(odometry_topic" in source
    assert '"publish_tf",\n                default_value="false"' in launch
    assert 'common.odom_frame: "odom"' in config
    assert 'common.body_frame: "livox_frame"' in config
    assert 'common.odometry_topic: "/fast_lio/odometry"' in config
    assert "publish.tf_en: false" in config
    assert 'DeclareLaunchArgument("odom_frame", default_value="odom")' in launch
    assert 'DeclareLaunchArgument("body_frame", default_value="livox_frame")' in launch
    assert '"common.odom_frame": LaunchConfiguration("odom_frame")' in launch
    assert '"common.body_frame": LaunchConfiguration("body_frame")' in launch


def test_effective_point_buffers_are_sized_before_writing():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

    assert "laserCloudOri->push_back(feats_down_body->points[i]);" in source
    assert "corr_normvect->push_back(normvec->points[i]);" in source
    assert "laserCloudOri->points[effct_feat_num] =" not in source
    assert "corr_normvect->points[effct_feat_num] =" not in source
    assert "laserCloudOri->reserve(static_cast<size_t>(feats_down_size));" in source
    assert "corr_normvect->reserve(static_cast<size_t>(feats_down_size));" in source
    assert "static_cast<size_t>(feats_down_size) > MAXN" in source


def test_runtime_diagnostics_bound_fixed_buffers_and_check_null_before_dereference():
    source = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

    assert "constexpr size_t MAX_RUNTIME_LOG_ENTRIES = MAXN;" in source
    assert "std::fill(std::begin(point_selected_surf), std::end(point_selected_surf), true);" in source
    assert "std::fill(std::begin(res_last), std::end(res_last), -1000.0f);" in source
    assert "feats_undistort == nullptr || feats_undistort->empty()" in source
    assert "time_log_counter >= MAX_RUNTIME_LOG_ENTRIES" in source
    assert "scan_count >= static_cast<int>(MAXN)" in source
    assert "if (!msg)" in source


def test_default_mapping_launch_uses_the_checked_in_real_mid360_config():
    launch = (FAST_LIO_ROOT / "launch" / "mapping.launch.py").read_text()
    config = (FAST_LIO_ROOT / "config" / "mid360.yaml").read_text()

    assert "mid360.yaml" in launch
    assert 'common.lid_topic: "/livox/lidar"' in config
    assert 'common.imu_topic: "/livox/imu"' in config
    assert 'common.odom_frame: "odom"' in config
    assert 'common.body_frame: "livox_frame"' in config
