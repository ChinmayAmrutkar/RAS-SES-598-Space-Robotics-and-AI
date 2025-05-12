from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for ArUco detection and landing."""
    
    package_dir = get_package_share_directory('terrain_mapping_drone_control')
    
    # Launch arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.8',
        description='Size of the ArUco marker in meters'
    )
    
    target_marker_id_arg = DeclareLaunchArgument(
        'target_marker_id',
        default_value='0',
        description='ID of the ArUco marker to target for landing'
    )
    
    # ArUco tracker node - using Python script directly
    aruco_tracker_node = Node(
        package='terrain_mapping_drone_control',
        executable='python3',
        name='aruco_tracker',
        output='screen',
        arguments=[
            os.path.join(get_package_share_directory('terrain_mapping_drone_control'), '..', '..', 
                         'lib', 'terrain_mapping_drone_control', 'aruco_tracker')
        ],
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size')
        }]
    )
    
    # Landing controller node - using Python script directly
    landing_controller_node = Node(
        package='terrain_mapping_drone_control',
        executable='python3',
        name='aruco_landing_controller',
        output='screen',
        arguments=[
            os.path.join(get_package_share_directory('terrain_mapping_drone_control'), '..', '..', 
                         'lib', 'terrain_mapping_drone_control', 'aruco_landing_controller')
        ],
        parameters=[{
            'target_marker_id': LaunchConfiguration('target_marker_id'),
            'target_altitude': 1.0,
            'landing_speed': 0.3,
            'position_tolerance': 0.05,
            'altitude_tolerance': 0.05
        }]
    )
    
    # Create and return launch description
    return LaunchDescription([
        marker_size_arg,
        target_marker_id_arg,
        aruco_tracker_node,
        landing_controller_node
    ])
