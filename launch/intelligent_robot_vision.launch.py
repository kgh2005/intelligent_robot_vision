from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('intelligent_robot_vision'),
        'model',
        'yolo11n.xml'
    )

    # Realsense 카메라 노드 (단일 노드)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        output='screen',
        parameters=[
            {'camera_name': 'camera'},
            {'publish_tf': True},
            {'align_depth.enable': True},
            {'enable_color': True},
            {'enable_depth': True},
            {'enable_sync': True},
            {'enable_pointcloud': True},
            {'depth_module.depth_profile': '848x480x30'},
            {'rgb_camera.color_profile': '848x480x30'},
        ]
    )

    detection_node = Node(
        package='intelligent_robot_vision',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[
            {'model_xml': model_path}
        ]
    )

    return LaunchDescription([
        realsense_node,
        detection_node
    ])
