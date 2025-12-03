import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 1. Include the Avia Driver (using the ASIG-X repo or standard driver)
    # Ensure this path matches your driver package name (livox_ros2_avia or livox_ros2_driver)
    livox_driver_pkg = get_package_share_directory('livox_ros2_avia')
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(livox_driver_pkg, 'launch', 'livox_lidar_launch.py'))
    )

    # 2. IMU Filter (Madgwick)
    # This takes raw accel/gyro and outputs a clean Quaternion
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,       # Avia has no magnetometer
            'publish_tf': False,    # We will publish our own custom TF
            'world_frame': 'nwu',   # Standard ROS frame
            'fixed_frame': 'world',
            'gain': 0.1             # Lower value = smoother but slower response
        }],
        remappings=[
            ('/imu/data_raw', '/livox/imu'), # Input: Raw Avia data
            ('/imu/data', '/imu/data')       # Output: Filtered data
        ]
    )

    # 3. Our Custom TF Broadcaster
    tf_broadcaster_node = Node(
        package='simple_pan_tilt',
        executable='broadcaster',
        name='pan_tilt_broadcaster'
    )

    # 4. RViz (Optional config)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('livox_ros2_avia'), 'config', 'livox_lidar.rviz')]
    )

    return LaunchDescription([
        livox_launch,
        imu_filter_node,
        tf_broadcaster_node,
        rviz_node
    ])