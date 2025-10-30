#!/usr/bin/env python3

import os
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_description, 'worlds', 'ball_world.sdf')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'robot_view.rviz')
    
    # Process xacro to get URDF
    robot_description_content = xacro.process_file(urdf_file).toxml()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content,
            'publish_default_positions': True,
            'default_positions': {
                'left_wheel_joint': 0.0,
                'right_wheel_joint': 0.0
            }
        }]
    )
    
    # Gazebo Sim - Start FIRST, alone
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file} --render-engine ogre'
        }.items()
    )
    
    # Spawn robot - Wait 5 seconds for Gazebo to start
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'my_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )
    
    # Bridges - Wait 6 seconds
    bridges = TimerAction(
        period=6.0,
        actions=[
            # Camera bridges (bidirectional)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
                name='camera_bridge',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
                name='camera_info_bridge',
            ),
            # cmd_vel bridge (bidirectional)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                name='cmd_vel_bridge',
            ),
            # Odometry bridge (bidirectional)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
                name='odometry_bridge',
            ),
            # TF bridges (Gazebo <-> ROS)
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
                name='tf_bridge',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
                name='tf_static_bridge',
            ),
        ]
    )
    
    # Ball Chaser - Wait 7 seconds
    ball_chaser = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ball_chaser',
                executable='ball_chaser_node',
                name='ball_chaser',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # RViz - Start LAST, wait 8 seconds for everything else
    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # Static transform: world -> odom (identity), for required TF chain
    static_world_to_odom = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_odom_broadcaster',
                arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
                output='screen'
            )
        ]
    )

    # TF view_frames - Wait 10 seconds
    view_frames = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        static_world_to_odom,
        bridges,
        ball_chaser,
        rviz,
        view_frames
    ])