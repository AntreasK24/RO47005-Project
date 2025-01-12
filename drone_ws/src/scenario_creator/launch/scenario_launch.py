from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
#        DeclareLaunchArgument('cylinder_centers', default_value='[(5,5),(10,10)]'),
#        DeclareLaunchArgument('cylinder_radii', default_value='[2,3]'),
DeclareLaunchArgument('repulsion_constant', default_value='1.0', description='The repulsion constant for the drone'),

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
<<<<<<< HEAD
            name='drone_mpc'
=======
            name='drone_mpc',
            parameters=[{'repulsion_constant': LaunchConfiguration('repulsion_constant')}],
>>>>>>> 5070deea4e8eb62b159f98c0a6469cc8eec9289d
        ),

        Node(
            package='trajectory_generator',
            executable='trajectory_publisher',
<<<<<<< HEAD
            name='trajectory_publisher'
        )
    ])
=======
            name='trajectory_generator'
        ),
    ])
>>>>>>> 5070deea4e8eb62b159f98c0a6469cc8eec9289d
