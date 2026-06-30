import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _text(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _bool_text(value):
    return str(value).lower() in {"1", "true", "yes", "on"}


def _launch_nodes(context, *args, **kwargs):
    use_pcap = _bool_text(_text(context, "use_p1_imu_pcap"))
    pcap_path = _text(context, "p1_imu_pcap_path")
    pcap_output_topic = _text(context, "p1_imu_pcap_output_topic")
    imu_input_topic = _text(context, "imu_input_topic")
    if use_pcap and not pcap_path:
        raise RuntimeError(
            "Point One IMU PCAP replay is enabled by default. "
            "Pass p1_imu_pcap_path:=/path/to/ins.pcap, or set "
            "use_p1_imu_pcap:=false to consume the upstream IMU ROS topic."
        )
    if use_pcap:
        imu_input_topic = pcap_output_topic
    elif not imu_input_topic:
        imu_input_topic = "/atlas/imu_calibrated"

    adapter_params = {
        "use_sim_time": _bool_text(_text(context, "use_sim_time")),
        "pose_input_topic": _text(context, "pose_input_topic"),
        "imu_input_topic": imu_input_topic,
        "publish_gnss_pose": _bool_text(_text(context, "publish_gnss_pose")),
        "summary_output_path": _text(context, "summary_output_path"),
        "imu_stamp_mode": _text(context, "imu_stamp_mode"),
        "imu_p1_sidecar_path": _text(context, "imu_p1_sidecar_path"),
        "imu_p1_sidecar_match_tolerance_sec": float(
            _text(context, "imu_p1_sidecar_match_tolerance_sec")
        ),
    }
    local_enu_origin = _text(context, "local_enu_origin")
    local_enu_origin_ttl_path = _text(context, "local_enu_origin_ttl_path")
    if local_enu_origin and local_enu_origin_ttl_path:
        raise RuntimeError("Set only one of local_enu_origin or local_enu_origin_ttl_path.")
    if local_enu_origin:
        adapter_params["local_enu_origin"] = local_enu_origin
        adapter_params["local_enu_origin_ttl_path"] = ""
    elif local_enu_origin_ttl_path:
        # Override the YAML default origin so the node sees exactly one source.
        adapter_params["local_enu_origin"] = ""
        adapter_params["local_enu_origin_ttl_path"] = local_enu_origin_ttl_path

    nodes = [
        Node(
            package="adapter",
            executable="adapter",
            name="adapter",
            output="screen",
            parameters=[_text(context, "params_file"), adapter_params],
        )
    ]

    if use_pcap:
        nodes.insert(
            0,
            Node(
                package="adapter",
                executable="p1_imu_pcap_replay_node.py",
                name="p1_imu_pcap_replay",
                output="screen",
                parameters=[
                    {
                        "pcap_path": pcap_path,
                        "output_topic": pcap_output_topic,
                        "clock_topic": _text(context, "p1_imu_pcap_clock_topic"),
                        "frame_id": _text(context, "p1_imu_pcap_frame_id"),
                        "pace_mode": _text(context, "p1_imu_pcap_pace_mode"),
                        "play_rate": float(_text(context, "p1_imu_pcap_play_rate")),
                        "max_clock_lag_sec": float(
                            _text(context, "p1_imu_pcap_max_clock_lag_sec")
                        ),
                        "max_messages": int(_text(context, "p1_imu_pcap_max_messages")),
                    }
                ],
            ),
        )

    return nodes


def generate_launch_description():
    share = get_package_share_directory("adapter")
    default_params = os.path.join(share, "config", "adapter.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("pose_input_topic", default_value="/atlas/pose_filtered"),
            DeclareLaunchArgument("imu_input_topic", default_value=""),
            DeclareLaunchArgument("imu_stamp_mode", default_value="auto"),
            DeclareLaunchArgument("imu_p1_sidecar_path", default_value=""),
            DeclareLaunchArgument("imu_p1_sidecar_match_tolerance_sec", default_value="0.02"),
            DeclareLaunchArgument("publish_gnss_pose", default_value="true"),
            DeclareLaunchArgument("local_enu_origin", default_value=""),
            DeclareLaunchArgument("local_enu_origin_ttl_path", default_value=""),
            DeclareLaunchArgument("summary_output_path", default_value=""),
            DeclareLaunchArgument("use_p1_imu_pcap", default_value="true"),
            DeclareLaunchArgument("p1_imu_pcap_path", default_value=""),
            DeclareLaunchArgument(
                "p1_imu_pcap_output_topic", default_value="/atlas/imu_calibrated_pcap"
            ),
            DeclareLaunchArgument("p1_imu_pcap_clock_topic", default_value="/clock"),
            DeclareLaunchArgument("p1_imu_pcap_frame_id", default_value="cg"),
            DeclareLaunchArgument("p1_imu_pcap_pace_mode", default_value="clock"),
            DeclareLaunchArgument("p1_imu_pcap_play_rate", default_value="1.0"),
            DeclareLaunchArgument("p1_imu_pcap_max_clock_lag_sec", default_value="1.0"),
            DeclareLaunchArgument("p1_imu_pcap_max_messages", default_value="0"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
