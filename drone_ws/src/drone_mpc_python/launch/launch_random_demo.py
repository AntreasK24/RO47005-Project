from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mpc_python',
            executable='drone_mpc_python',
            name='drone_mpc_python',
        ),

        Node(
            package='drone_simulator',
            executable='drone_simulator',
            name='drone_simulator'
        ),

        Node(
            package='trajectory_generator',
            executable='trajectory_publisher',
            name='trajectory_publisher'
        )
    ])
