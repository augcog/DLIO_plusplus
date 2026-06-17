from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("dlio_input_adapter")
    default_params = os.path.join(share, "config", "dlio_input_adapter.yaml")

    params = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    utm_origin = LaunchConfiguration("utm_origin")
    utm_origin_output_path = LaunchConfiguration("utm_origin_output_path")
    t_world_utm_path = LaunchConfiguration("T_world_utm_path")
    imu_stamp_mode = LaunchConfiguration("imu_stamp_mode")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("utm_origin", default_value=""),
        DeclareLaunchArgument("utm_origin_output_path", default_value=""),
        DeclareLaunchArgument("T_world_utm_path", default_value=""),
        DeclareLaunchArgument("imu_stamp_mode", default_value="auto"),
        Node(
            package="dlio_input_adapter",
            executable="dlio_input_adapter_node",
            name="dlio_input_adapter",
            output="screen",
            parameters=[
                params,
                {
                    "use_sim_time": use_sim_time,
                    "utm_origin": utm_origin,
                    "utm_origin_output_path": utm_origin_output_path,
                    "T_world_utm_path": t_world_utm_path,
                    "imu_stamp_mode": imu_stamp_mode,
                },
            ],
        ),
    ])
