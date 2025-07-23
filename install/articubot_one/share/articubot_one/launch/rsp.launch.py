import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get path to package
    pkg_path = get_package_share_directory('articubot_one')
    
    # Xacro file
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Config files
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_path, 'config', 'slam.rviz')

    return LaunchDescription([
        # Declare 'use_sim_time' argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
                                                                                                                                            description='Use simulation (Gazebo) clock if true'
        ),
        # Static transform from odom to base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_base_link',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        # ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description}
            ]
        ),
        # RPLIDAR Node
        # RViz2
        # Node(
        #     package='rviz2',                     # Gọi gói rviz2
        #     executable='rviz2',                  # Thực thi rviz2
        #     name='rviz2',                        # Tên node
        #     arguments=['-d', rviz_config_file], # Mở file cấu hình rviz
        #     parameters=[{'use_sim_time': use_sim_time}], # Dùng thời gian mô phỏng (nếu cần)
        #     output='screen'                      # Hiển thị log trên terminal
        # ),
        
    ])
