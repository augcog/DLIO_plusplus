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
    current_pkg = FindPackageShare('gicp_localization')

    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/luminar_front/points')
    # All-P1 single-source design:
    #   imu_topic     = /gps_p1/imu              (Atlas imu_calibrated)
    #   gt_odom_topic = /gps_p1/filtered_odom    (Atlas FusionEngine INS)
    #   imu_frame / base_frame = "gps_antenna_top"
    #
    # Atlas projects its IMU output AND its INS pose solution to the primary
    # GNSS antenna phase centre via firmware lever-arm, so every comparison
    # the node performs lives at the same body reference -- no TF lever-arm
    # correction needed anywhere, no second GNSS vendor, no novatel_oem7_msgs
    # dependency. The RTK quality gate inspects msg->pose.covariance on the
    # gt_odom message itself, so there is no separate /bestgnsspos
    # subscription -- the gate is self-contained in callbackGtOdom.
    imu_topic = LaunchConfiguration('imu_topic', default='/gps_p1/imu')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    gt_odom_topic = LaunchConfiguration('gt_odom_topic', default='/gps_p1/filtered_odom_map')
    imu_only = LaunchConfiguration('imu_only', default='false')
    publish_tf = LaunchConfiguration('publish_tf', default='false')
    deskew = LaunchConfiguration('deskew', default='false')
    crop_size = LaunchConfiguration('crop_size', default='80.0')
    sensor_type = LaunchConfiguration('sensor_type', default='luminar')
    lidar_concat_enabled = LaunchConfiguration('lidar_concat_enabled', default='true')
    gt_recovery_enabled = LaunchConfiguration('gt_recovery_enabled', default='true')
    gt_rejection_enabled = LaunchConfiguration('gt_rejection_enabled', default='true')
    verbose = LaunchConfiguration('verbose', default='false')
    verbose_scan_log = LaunchConfiguration('verbose_scan_log', default='false')
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
                    '99 Hz, lever-arm-projected by Atlas firmware to the '
                    'primary antenna phase centre gps_antenna_top). Stays '
                    'in sync with base_frame=gps_antenna_top in '
                    'localization.yaml.')
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value=odom_topic, description='Odometry topic name (for initialization)')
    declare_gt_odom_topic_arg = DeclareLaunchArgument(
        'gt_odom_topic', default_value=gt_odom_topic,
        description='Ground-truth odometry topic for init / divergence cross-check / GT-recovery '
                    'snap. MUST be in the map frame of the loaded PCD map. The prepped bags carry '
                    '/gps_p1/filtered_odom in the "utm" frame; run '
                    'gicp_localization/scripts/utm_to_map_odom.py (with the map dump\'s '
                    'T_world_utm.txt) to produce the default /gps_p1/filtered_odom_map. '
                    'child_frame_id is gps_antenna_top (matches base_frame). '
                    'Do NOT point this at /localization/global/odom (cg frame) without also changing '
                    'localization/base_frame to cg, or the cross-check baseline will be biased by '
                    '~0.39 m and applyInitialPose will seed the state offset by the same amount.')
    declare_imu_only_arg = DeclareLaunchArgument(
        'imu_only', default_value=imu_only,
        description='If true, disable GICP and run IMU-only propagation')
    declare_publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value=publish_tf,
        description='If true, publish map -> base_frame TF. Useful for RViz views/displays.')
    declare_deskew_arg = DeclareLaunchArgument(
        'deskew', default_value=deskew,
        description='Override dlio/deskew for validation replays.')
    declare_crop_size_arg = DeclareLaunchArgument(
        'crop_size', default_value=crop_size,
        description='Override dlio/preprocessing/cropBoxFilter/size. Use >=1000 to skip crop.')
    declare_sensor_type_arg = DeclareLaunchArgument(
        'sensor_type', default_value=sensor_type,
        description='Override localization/sensor_type for timestamp handling.')
    declare_lidar_concat_enabled_arg = DeclareLaunchArgument(
        'lidar_concat_enabled', default_value=lidar_concat_enabled,
        description='Override localization/lidar_concat/enabled.')
    declare_gt_recovery_enabled_arg = DeclareLaunchArgument(
        'gt_recovery_enabled', default_value=gt_recovery_enabled,
        description='Override localization/gt_recovery/enable.')
    declare_gt_rejection_enabled_arg = DeclareLaunchArgument(
        'gt_rejection_enabled', default_value=gt_rejection_enabled,
        description='Override localization/gt_rejection/enable.')
    declare_verbose_arg = DeclareLaunchArgument(
        'verbose', default_value=verbose,
        description='Override localization/verbose.')
    declare_verbose_scan_log_arg = DeclareLaunchArgument(
        'verbose_scan_log', default_value=verbose_scan_log,
        description='Override localization/debug/verbose_scan_log.')
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
    declare_utm_transform_path_arg = DeclareLaunchArgument(
        'utm_transform_path', default_value='',
        description='Path to the GLIM dump\'s T_world_utm.txt for the map in use. '
                    'Enables the gicp/localization/*_utm output topics '
                    '(overrides localization.yaml when non-empty)')

    localization_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'localization.yaml'])

    # Publish the full vehicle URDF via robot_state_publisher. This provides the
    # real base_link -> luminar_front and base_link -> gps_bottom/imu_bottom
    # transforms from the URDF, replacing the hand-maintained static TFs.
    def make_robot_state_publisher(context):
        urdf_file = LaunchConfiguration('urdf_path').perform(context).strip()
        # If no explicit path was given, walk up from this launch file to find
        # av24.urdf.  Works from both the source tree and the colcon install tree.
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
        utm_path_value = LaunchConfiguration('utm_transform_path').perform(context).strip()
        child_frame_value = LaunchConfiguration('child_frame').perform(context).strip()
        params = [
            localization_yaml_path,
            {'localization/lidar_frame': child_frame_value},
            {'localization/imu_only': LaunchConfiguration('imu_only')},
            {'localization/publish_tf': LaunchConfiguration('publish_tf')},
            {'dlio/deskew': LaunchConfiguration('deskew')},
            {'dlio/preprocessing/cropBoxFilter/size': LaunchConfiguration('crop_size')},
            {'localization/sensor_type': LaunchConfiguration('sensor_type')},
            {'localization/lidar_concat/enabled': LaunchConfiguration('lidar_concat_enabled')},
            {'localization/gt_recovery/enable': LaunchConfiguration('gt_recovery_enabled')},
            {'localization/gt_rejection/enable': LaunchConfiguration('gt_rejection_enabled')},
            {'localization/verbose': LaunchConfiguration('verbose')},
            {'localization/debug/verbose_scan_log': LaunchConfiguration('verbose_scan_log')},
        ]
        if map_path_value:
            params.append({'localization/map_path': map_path_value})
        if utm_path_value:
            params.append({'localization/utm_transform_path': utm_path_value})

        node = Node(
            package='gicp_localization',
            executable='gicp_localization_node',
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
            [FindPackageShare('gicp_localization'), 'cfg', 'localization.yaml']
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
            name='gicp_localization_rviz',
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
        declare_publish_tf_arg,
        declare_deskew_arg,
        declare_crop_size_arg,
        declare_sensor_type_arg,
        declare_lidar_concat_enabled_arg,
        declare_gt_recovery_enabled_arg,
        declare_gt_rejection_enabled_arg,
        declare_verbose_arg,
        declare_verbose_scan_log_arg,
        declare_urdf_path_arg,
        declare_parent_frame_arg,
        declare_child_frame_arg,
        declare_map_path_arg,
        declare_utm_transform_path_arg,
        OpaqueFunction(function=make_robot_state_publisher),
        OpaqueFunction(function=make_localization_node),
        OpaqueFunction(function=make_rviz_node),
    ])
