from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    config_husky_ekf = PathJoinSubstitution(
        [FindPackageShare('husky_control'),
        'config',
        'localization.yaml'],
    )
    
    
      # Start robot localization using an Extended Kalman filter...map->odom transform
    start_robot_localization_global_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_map',
    output='screen',
    parameters=[config_husky_ekf],
    remappings=[('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose')])
    ld.add_action(start_robot_localization_global_cmd)

    # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_odom',
    output='screen',
    parameters=[config_husky_ekf],
    remappings=[('odometry/filtered', 'odometry/local'),
                ('/set_pose', '/initialpose')])
              
    ld.add_action(start_robot_localization_local_cmd)

    start_navsat_transform_cmd = Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform',
    output='screen',
    parameters=[config_husky_ekf],
    remappings=[('imu', 'imu/data'),
                ('gps/fix', 'gnss1/fix'), 
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global')])
    ld.add_action(start_navsat_transform_cmd)

    primary_imu_enable = EnvironmentVariable('CPR_IMU', default_value='false')

    if (primary_imu_enable.perform(lc)) == 'true':
        config_imu_filter = PathJoinSubstitution(
            [FindPackageShare('husky_control'),
            'config',
            'imu_filter.yaml'],
        )
        node_imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[config_imu_filter]
        )
        ld.add_action(node_imu_filter)

    return ld
