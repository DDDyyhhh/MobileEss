"""Start FAST-LIO against the real Livox CustomMsg/IMU topics.

The real parameter file is intentionally supplied outside the repository.  It
must contain field-calibrated LiDAR/IMU extrinsics; the simulation YAML is not
used as a fallback.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=EnvironmentVariable(
                    "FAST_LIO_REAL_CONFIG",
                    default_value="/etc/mower/fast_lio/mid360_real.yaml",
                ),
                description=(
                    "Field-calibrated FAST-LIO YAML; no simulation fallback is used"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("body_frame", default_value="livox_frame"),
            DeclareLaunchArgument(
                "odometry_topic", default_value="/fast_lio/odometry"
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="Publish FAST-LIO odometry TF; keep false when an EKF owns odom -> base_link",
            ),
            DeclareLaunchArgument(
                "runtime_mapping_diagnostics", default_value="false"
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "common.odom_frame": LaunchConfiguration("odom_frame"),
                        "common.body_frame": LaunchConfiguration("body_frame"),
                        "common.odometry_topic": LaunchConfiguration(
                            "odometry_topic"
                        ),
                        "publish.tf_en": LaunchConfiguration("publish_tf"),
                        "runtime_mapping_diagnostics": LaunchConfiguration(
                            "runtime_mapping_diagnostics"
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
