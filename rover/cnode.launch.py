import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ================== GAZEBO ==================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ])
    )

    # ================== XACRO ==================
    pkg_share = get_package_share_directory('w_rover')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}
    controller_config = os.path.join(pkg_share, 'config', 'rover_controllers.yaml')

    # ================== ROBOT STATE PUBLISHER ==================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # ================== SPAWN ENTITY ==================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover', '-z', '0.1'],
        output='screen'
    )

    
    #controller_manager = Node(
    #    package='controller_manager',
    #    executable='ros2_control_node',
    #    parameters=[params, controller_config],
    #    output='screen',
    #)

    # ================== LOAD CONTROLLERS ==================
    # Wait for Gazebo to start its controller manager, then load controllers
    load_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ================== RVIZ ==================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        #TimerAction(
        #    period=3.0,
        #    actions=[controller_manager]
        #),
        TimerAction(
            period=5.0,
            actions=[load_joint_state]
        ),
        TimerAction(
            period=6.0,
            actions=[load_diff_drive]
        ),
        rviz
    ])
