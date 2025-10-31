#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
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
    # This node publishes the static transforms from the URDF (e.g., base_link -> camera_link)
    # and the dynamic transforms for the wheels based on /joint_states.
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
    
    # ONE BRIDGE TO RULE THEM ALL - Wait 6 seconds
    # This single node bridges all necessary topics between Gazebo and ROS 2.
    # The syntax '[gz.msgs...' means Gazebo -> ROS.
    # The syntax ']gz.msgs...' means ROS -> Gazebo.
    bridges = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # Command velocity (ROS -> GZ)
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    # Odometry (GZ -> ROS)
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # TF (GZ -> ROS) - Publishes odom->base_footprint
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    # Joint States (GZ -> ROS) - Publishes wheel rotations
                    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    # Camera Image (GZ -> ROS)
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    # Camera Info (GZ -> ROS)
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
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

    # Static transform: camera_link -> sensor frame used by Gazebo messages
    static_camera_frame = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_sensor_frame_broadcaster',
                arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'my_robot/base_footprint/camera'],
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
    
    # The final list of actions to launch.
    # Note: The custom joint_state_publisher and odom_to_tf nodes have been removed.
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        static_world_to_odom,
        static_camera_frame,
        bridges,
        ball_chaser,
        rviz,
        view_frames
    ])
