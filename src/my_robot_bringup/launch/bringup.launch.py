import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


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
    import xacro
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
            'publish_default_positions': True,
            'default_positions': {
                'left_wheel_joint': 0.0,
                'right_wheel_joint': 0.0
            }
        }]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file} --render-engine ogre'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
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

    # Static transform: world -> odom (identity), for required TF chain
    static_world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )

    # Static transform: camera_link -> sensor frame used by Gazebo messages
    static_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_sensor_frame_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'my_robot/base_footprint/camera'],
        output='screen'
    )
    
    # Bridge for cmd_vel (ROS <-> Gazebo)
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )
    
    # Bridge for odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen',
        remappings=[
            ('/odometry', '/odom')
        ]
    )
    
    # Bridge for camera
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    
    # Bridge for camera info
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Bridge TF topics from Gazebo to ROS
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen'
    )
    bridge_tf_static = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # TF2 view_frames
    view_frames = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
        output='screen'
    )
    
    # Ball Chaser Node
    ball_chaser = Node(
        package='ball_chaser',
        executable='ball_chaser_node',
        name='ball_chaser',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        static_world_to_odom,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        bridge_cmd_vel,
        bridge_odom,
        bridge_camera,
        bridge_camera_info,
        bridge_tf,
        bridge_tf_static,
        static_camera_frame,
        rviz,
        view_frames,
        ball_chaser
    ])
