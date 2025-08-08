import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('point_lio')
    this_package_path = get_package_share_directory('localization_qn')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(this_package_path, 'config', 'localization_rviz.rviz')

    localization_qn_path = get_package_share_directory('localization_qn')
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
        'config_file', default_value='unitree_L1.yaml',
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

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([package_path, 'config', 'unilidar_l1.yaml']),
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,  # Options: 1, 3
            'space_down_sample': True,
            'filter_size_surf': 0.1,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.1,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # Node definition for laserMapping with Point-LIO
    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    
    fastlio_qn_node = Node(
        package='localization_qn',
        executable='localization_qn_node',
        parameters=[PathJoinSubstitution([localization_qn_config_path, 'config.yaml'])],
        remappings=[('/Odometry', 'aft_mapped_to_init'), ('/cloud_registered', cloud_registered_topic)],
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

    ld.add_action(point_lio)
    ld.add_action(fastlio_qn_node)
    ld.add_action(rviz_node)

    return ld
