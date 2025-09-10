from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('intelligent_robot_vision'),
        'model',
        'best.xml'
    )

    tilt_info = os.path.join(
        get_package_share_directory('intelligent_robot_vision'),
        'config',
        'tilt_config.yaml'
    )

    # Realsense 카메라 노드 (단일 노드)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        output='screen',
        parameters=[
            {'camera_name': 'camera'},
            {'publish_tf': False},
            {'publish_extrinsics': False},
            {'align_depth.enable': True},
            {'depth.align_to': 'color'},
            {'enable_color': True},
            {'enable_depth': True},
            {'enable_infra1': False},
            {'enable_infra2': False},
            {'enable_sync': False},
            {'enable_pointcloud': False},
            {'enable_depth_metadata': False},
            {'enable_color_metadata': False},
            {'depth_module.depth_profile': '640x480x15'},
            {'rgb_camera.color_profile': '640x480x15'},
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

    refiner_node = Node(
        package='intelligent_robot_vision',
        executable='refiner_node',
        name='refiner_node',
        output='screen'
    )

    pan_tilt_node = Node(
        package='intelligent_robot_vision',
        executable='pan_tilt_node',
        name='pan_tilt_node',
        output='screen',
        parameters=[tilt_info]
    )

    return LaunchDescription([
        realsense_node,
        detection_node,
        refiner_node,
        pan_tilt_node
    ])
