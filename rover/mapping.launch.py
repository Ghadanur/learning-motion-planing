from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # --- LEFT image_proc (rectify) ---
        Node(
            package='image_proc',
            executable='image_proc',
            name='left_rect_proc',
            remappings=[
                ('image', '/left/left/image_raw'),
                ('camera_info', '/left/left/camera_info'),
                ('image_rect', '/left/left/image_rect')
            ],
            parameters=[{"approximate_sync": True}]
        ),

        # --- RIGHT image_proc (rectify) ---
        Node(
            package='image_proc',
            executable='image_proc',
            name='right_rect_proc',
            remappings=[
                ('image', '/right/right/image_raw'),
                ('camera_info', '/right/right/camera_info'),
                ('image_rect', '/right/right/image_rect')
            ],
            parameters=[{"approximate_sync": True}]
        ),

        # --- Stereo Disparity Node ---
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            remappings=[
                ('left/image_rect', '/left/left/image_rect'),
                ('right/image_rect', '/right/right/image_rect'),
                ('left/camera_info', '/left/left/camera_info'),
                ('right/camera_info', '/right/right/camera_info'),
                ('disparity', '/stereo/disparity')
            ]
        ),

        # --- Stereo PointCloud Node ---
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            name='stereo_pointcloud_node',
            remappings=[
                ('left/image_rect_color', '/left/left/image_rect'),
                ('right/image_rect_color', '/right/right/image_rect'),
                ('left/camera_info', '/left/left/camera_info'),
                ('right/camera_info', '/right/right/camera_info'),
                ('disparity', '/stereo/disparity'),
                ('points2', '/stereo/points2')
            ]
        ),

        # --- PointCloud → LaserScan ---
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2_to_scan',
            remappings=[
                ('cloud_in', '/stereo/points2'),
                ('scan', '/camera/scan')
            ],
            parameters=[{
                "target_frame": "base_link",
                "min_height": -1.0,
                "max_height": 1.0,
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.005,
                "range_min": 0.2,
                "range_max": 10.0
            }]
        )
    ])
