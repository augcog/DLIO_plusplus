#
#   Copyright (c)
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare('gicp_plusplus')

    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/luminar_front/points')
    # All-P1 single-source design:
    #   imu_topic     = /gps_p1/imu              (Atlas imu_calibrated)
    #   gt_odom_topic = /gps_p1/filtered_odom    (Atlas FusionEngine INS)
    #   imu_frame / base_frame = "gps_antenna_top"
    #
    # Atlas outputs its INS pose/position at the primary GNSS antenna phase
    # centre; its IMU stream is body-axis-rotated but device-located (NOT
    # antenna-projected -- FusionEngine Spec 3.4.1). We run base_frame and
    # imu_frame both at gps_antenna_top, so pose comparisons are exact and the
    # IMU's small device->antenna accel lever arm is deliberately dropped --
    # no TF lever-arm correction anywhere, no second GNSS vendor, no
    # novatel_oem7_msgs dependency. The RTK quality gate inspects
    # msg->pose.covariance on the
    # gt_odom message itself, so there is no separate /bestgnsspos
    # subscription -- the gate is self-contained in callbackGtOdom.
    imu_topic = LaunchConfiguration('imu_topic', default='/gps_p1/imu')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    gt_odom_topic = LaunchConfiguration('gt_odom_topic', default='/gps_p1/filtered_odom')
    imu_only = LaunchConfiguration('imu_only', default='false')
    urdf_path = LaunchConfiguration(
        'urdf_path',
        default='')
    parent_frame = LaunchConfiguration('parent_frame', default='base_link')
    child_frame = LaunchConfiguration('child_frame', default='luminar_front')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value=rviz, description='Launch RViz')
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic', default_value=pointcloud_topic, description='Pointcloud topic name')
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value=imu_topic,
        description='IMU topic name. Default /gps_p1/imu (Point One Atlas '
                    'imu_calibrated: sensor-calibrated, gravity present, '
                    '99 Hz, body-axis-rotated but device-located -- NOT '
                    'antenna-projected). Tagged gps_antenna_top to stay in '
                    'sync with base_frame in localization.yaml; the small '
                    'device->antenna accel lever arm is dropped.')
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value=odom_topic, description='Odometry topic name (for initialization)')
    declare_gt_odom_topic_arg = DeclareLaunchArgument(
        'gt_odom_topic', default_value=gt_odom_topic,
        description='Ground-truth odometry topic for init / divergence cross-check / GT-recovery snap. '
                    'Default /gps_p1/filtered_odom -- Atlas FusionEngine INS solution, '
                    'header.frame_id="map", child_frame_id="gps_antenna_top" (matches base_frame). '
                    'Do NOT point this at /localization/global/odom (cg frame) without also changing '
                    'localization/base_frame to cg, or the cross-check baseline will be biased by '
                    '~0.39 m and applyInitialPose will seed the state offset by the same amount.')
    declare_imu_only_arg = DeclareLaunchArgument(
        'imu_only', default_value=imu_only,
        description='If true, disable GICP and run IMU-only propagation')
    declare_urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value=urdf_path,
        description='Absolute path to the vehicle URDF used by robot_state_publisher '
                    '(provides base_link -> {luminar_front, gps_bottom, imu_bottom, ...} TFs)')
    declare_parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value=parent_frame,
        description='Parent frame of the LiDAR sensor in the URDF')
    declare_child_frame_arg = DeclareLaunchArgument(
        'child_frame', default_value=child_frame,
        description='LiDAR sensor frame (must match incoming PointCloud2 header.frame_id and URDF link)')
    declare_map_path_arg = DeclareLaunchArgument(
        'map_path', default_value='',
        description='Path to PCD map file for localization (overrides localization.yaml when non-empty)')

    localization_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'localization.yaml'])

    # Publish the full vehicle URDF via robot_state_publisher. This provides the
    # real base_link -> luminar_front and base_link -> gps_bottom/imu_bottom
    # transforms from the URDF, replacing the hand-maintained static TFs.
    # Resolve av24.urdf to an ABSOLUTE path so both robot_state_publisher AND the
    # localization node's lidar_concat extrinsic resolution find it CWD-independently.
    # If no explicit urdf_path arg is given, walk up from this launch file: av24.urdf
    # now lives in the gicp_plusplus package root (and also at the repo root),
    # so the walk finds it in both the source tree and the colcon install tree.
    def resolve_urdf_path(context):
        urdf_file = LaunchConfiguration('urdf_path').perform(context).strip()
        if not urdf_file:
            d = os.path.dirname(os.path.abspath(__file__))
            for _ in range(10):
                candidate = os.path.join(d, 'av24.urdf')
                if os.path.isfile(candidate):
                    urdf_file = candidate
                    break
                d = os.path.dirname(d)
        if not os.path.isfile(urdf_file):
            raise RuntimeError(
                f"URDF file not found at '{urdf_file}'. "
                f"Pass a different path with urdf_path:=<abs-path>.")
        return urdf_file

    def make_robot_state_publisher(context):
        urdf_file = resolve_urdf_path(context)
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
        node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
        return [node]

    # GICP Localization Node
    def make_localization_node(context):
        map_path_value = LaunchConfiguration('map_path').perform(context).strip()
        child_frame_value = LaunchConfiguration('child_frame').perform(context).strip()
        # Same av24.urdf the robot_state_publisher uses: hand the localization node
        # the resolved ABSOLUTE path so lidar_concat resolves aux extrinsics from the
        # URDF (single source of truth) instead of relying on CWD or the static fallback.
        urdf_file = resolve_urdf_path(context)
        params = [
            localization_yaml_path,
            {'localization/lidar_frame': child_frame_value},
            {'localization/imu_only': LaunchConfiguration('imu_only')},
            {'localization/lidar_concat/urdf_path': urdf_file},
        ]
        if map_path_value:
            params.append({'localization/map_path': map_path_value})

        node = Node(
            package='gicp_plusplus',
            executable='gicp_plusplus_node',
            output='screen',
            parameters=params,
            remappings=[
                ('pointcloud', pointcloud_topic),
                ('imu', imu_topic),
                ('odom', odom_topic),
                ('gt_odom', gt_odom_topic),
                ('localized_pose', 'gicp/localization/pose'),
                ('localized_odom', 'gicp/localization/odom'),
                ('localized_path', 'gicp/localization/path'),
                ('map', 'gicp/localization/map'),
            ],
        )
        return [node]

    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'localization.rviz'])

    def make_rviz_node(context):
        yaml_path = PathJoinSubstitution(
            [FindPackageShare('gicp_plusplus'), 'cfg', 'localization.yaml']
        ).perform(context)
        with open(yaml_path, 'r') as f:
            ros_params = yaml.safe_load(f).get('/**', {}).get('ros__parameters', {})
        map_frame = ros_params.get('localization/map_frame', 'map')
        base_frame = ros_params.get('localization/base_frame', 'base_link')

        template_path = rviz_config_path.perform(context)
        with open(template_path, 'r') as f:
            rviz_content = f.read()
        rviz_content = rviz_content.replace('__MAP_FRAME__', map_frame)
        rviz_content = rviz_content.replace('__BASE_FRAME__', base_frame)

        tmp = tempfile.NamedTemporaryFile(suffix='.rviz', mode='w', delete=False)
        tmp.write(rviz_content)
        tmp.close()

        return [Node(
            package='rviz2',
            executable='rviz2',
            name='gicp_plusplus_rviz',
            arguments=['-d', tmp.name],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        )]

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_odom_topic_arg,
        declare_gt_odom_topic_arg,
        declare_imu_only_arg,
        declare_urdf_path_arg,
        declare_parent_frame_arg,
        declare_child_frame_arg,
        declare_map_path_arg,
        OpaqueFunction(function=make_robot_state_publisher),
        OpaqueFunction(function=make_localization_node),
        OpaqueFunction(function=make_rviz_node),
    ])
