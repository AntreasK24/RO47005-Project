'''

This example should show how to launch two nodes with 
two different virtual python environments (in our case
acados and gym-pybullet-drones)


'''




import os
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def launch_node_with_virtualenv(env_name, node_name, package_name):
    """
    Helper function to launch a ROS 2 node with a specific virtual environment activated.
    
    :param env_name: The name of the virtual environment directory
    :param node_name: The name of the ROS 2 node to launch
    :param package_name: The ROS 2 package where the node is located
    """
    # Create the command to activate the virtual environment and run the node
    activate_env = f"source {env_name}/bin/activate && "
    run_node_cmd = f"ros2 run {package_name} {node_name}"

    # Combine them to form the full shell command
    full_cmd = activate_env + run_node_cmd

    # Return the ExecuteProcess action to launch the node
    return ExecuteProcess(
        cmd=[full_cmd],
        shell=True,
        output='screen'
    )

def generate_launch_description():
    """
    Launch description for launching ROS 2 nodes with specific virtual environments.
    """

    # Node 1 (acados-related node)
    node1_virtualenv = os.path.expanduser('$HOME/acados-env')  # Path to acados virtualenv
    node1_package_name = 'acados_package'  # Replace with your actual ROS 2 package name
    node1_name = 'acados_node'  # Replace with your actual node name
    
    # Node 2 (gym-pybullet-drones-related node)
    node2_virtualenv = os.path.expanduser('$HOME/gym-pybullet-drones-env')  # Path to gym-pybullet-drones virtualenv
    node2_package_name = 'gym_pybullet_drones_package'  # Replace with your actual ROS 2 package name
    node2_name = 'gym_pybullet_drones_node'  # Replace with your actual node name

    return LaunchDescription([
        # Launch Node 1 (acados-related node) with the appropriate virtual environment
        launch_node_with_virtualenv(node1_virtualenv, node1_name, node1_package_name),
        
        # Launch Node 2 (gym-pybullet-drones-related node) with the appropriate virtual environment
        launch_node_with_virtualenv(node2_virtualenv, node2_name, node2_package_name),
    ])
