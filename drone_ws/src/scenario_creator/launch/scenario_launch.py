from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
#        DeclareLaunchArgument('cylinder_centers', default_value='[(5,5),(10,10)]'),
#        DeclareLaunchArgument('cylinder_radii', default_value='[2,3]'),
DeclareLaunchArgument('repulsion_constant', default_value='5.0', description='The repulsion constant for the drone'),

        Node(
            package='global_planner',
            executable='constraint_node',
            name='constraint_node',
#            parameters=[{
#                'cylinder_centers': LaunchConfiguration('cylinder_centers'),
#                'cylinder_radii': LaunchConfiguration('cylinder_radii')
#            }]
        ),

        Node(
            package='scenario_creator',
            executable='drone_setup',
            name='drone_setup'
        ),

        Node(
            package='drone_mpc_python',
            executable='drone_mpc_python',
            name='drone_mpc',
            parameters=[{'repulsion_constant': LaunchConfiguration('repulsion_constant')}],
        )
    ])