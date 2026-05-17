import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Chỉ định rõ ràng lấy cấu hình từ não bộ 'slam_uet'
    pkg_share = get_package_share_directory('slam_uet')   
    
    lua_config_path = os.path.join(pkg_share, 'config')
    lua_config_file = 'assem3_cartographer3d.lua'

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', lua_config_path,
                '-configuration_basename', lua_config_file
            ],
            remappings=[
                # Đã XÓA remapping của points2_1 và points2_2. 
                # Cartographer sẽ tự động đọc thẳng topic /points2 do URDF phát ra.
                ('imu', '/imu'),
                ('odom', '/odom'), # Giữ lại để đảm bảo kết nối chắc chắn với diff_drive
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])