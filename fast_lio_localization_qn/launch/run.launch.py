import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    localization_qn_path = get_package_share_directory('fast_lio_localization_qn')
    localization_qn_config_path = os.path.join(
        localization_qn_path, 'config') 

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    odom_topic = LaunchConfiguration('odom_topic')
    cloud_registered_topic = LaunchConfiguration('cloud_registered_topic')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='ouster64.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic', default_value='/Odometry',
        description='Odom topic'
    )
    declare_cloud_registered_topic_cmd = DeclareLaunchArgument(
        'cloud_registered_topic', default_value='/cloud_registered',
        description='Cloud registered topic'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    
    fastlio_qn_node = Node(
        package='fast_lio_localization_qn',
        executable='fast_lio_localization_qn_node',
        parameters=[PathJoinSubstitution([localization_qn_config_path, 'config.yaml'])],
        remappings=[('/Odometry', odom_topic), ('/cloud_registered', cloud_registered_topic)],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_odom_topic_cmd)
    ld.add_action(declare_cloud_registered_topic_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(fastlio_qn_node)
    ld.add_action(rviz_node)

    return ld
