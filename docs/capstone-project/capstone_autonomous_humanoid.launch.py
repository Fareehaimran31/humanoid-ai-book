"""Launch file for the complete capstone autonomous humanoid system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='humanoid_robot',
            description='Robot model to use'
        ),

        # Navigation system (Nav2)
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),

        # Autonomous humanoid system node
        Node(
            package='robot_capstone_system',
            executable='autonomous_humanoid_node',
            name='autonomous_humanoid',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_model': LaunchConfiguration('robot_model')}
            ],
            remappings=[
                ('/camera/rgb/image_raw', '/camera/image_raw'),
                ('/scan', '/lidar/scan'),
            ]
        ),

        # Perception system node
        Node(
            package='robot_capstone_system',
            executable='perception_node',
            name='perception_system',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # AI brain node
        Node(
            package='robot_capstone_system',
            executable='ai_brain_node',
            name='ai_brain',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # VLA system node
        Node(
            package='robot_capstone_system',
            executable='vla_system_node',
            name='vla_system',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # Isaac Lab integration node (if running with Isaac Sim)
        Node(
            package='isaac_ros_bridges',
            executable='isaac_ros_bridge',
            name='isaac_ros_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        )
    ])