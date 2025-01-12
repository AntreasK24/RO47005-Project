# RO47005-Project


| Student Name     | Student Number     |
|--------------|--------------|
| Antreas Kourris | 6284213 |
| Jonah Eggenkemper | 6286933 |
| Manas Reddy Ramidi	 | 6191258​ |
| Mukil Saravanan | 6195474​ |

This repository hosts the files used for our groups final project for RO47005 Path Planning and Decision Making



# Installation of the Environment

## Using our custom Singularity Container 

### Building the container

To get our code download the submitted .zip file, or clone the repository from [here](https://github.com/AntreasK24/RO47005-Project).

Navigate to ´./project_assets/singularity´ and check for the file ´singularity_RO47005_drone_ws_V03.def´. Now run the following command to create the singularity container ´ros2_humble_acados_drones.sif´ in your home directory.

´´´
sudo singularity build <path-to-file>/singularity_RO47005_drone_ws_V03.def ~/ros2_humble_acados_drones.sif
´´´

To run the container just run the following command. If you do not have a NVIDIA GPU remove the `--nv` flag.

```
singularity shell --nv <path-to-file>/ros2_humble_acados_drones.sif 
```

### Building the drone workspace

In our provided repository you will already find a prepared workspace called `drone_ws`. Once you started the singularity container, navigate to `drone_ws` and run:

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

The workspace should now be set up and ready to go. 


## Running the scenarios

After the workspace has been built with 

```
colcon build
```

The two different scenarios can be run by using the followig commands:

In order to start the first scenario, run:

```
 ros2 launch drone_mpc_python  launch_random_demo.py
```

This will launch the scenario where the drone will receive random target positions in a 3D world with randomly spawing spherical objects. The drone will navigate to the target positions using an MPC while avoiding the obstacles

![scenario1](./project_assets/Images/scenario1.gif)

In order to launch the second scenario run the following command:

```
ros2 launch scenario_creator scenario_launch.py  
```

This scenario features a building. Inside the building the drone must navigate to random positions while avoiding collision with the pillars.

![scenario2](./project_assets/Images/scenario2-1.gif)

## Explanation of packages

This repository relies mainly on 4 packages. A brief description of each package follows below.

### drone_mpc_python

This package houses the model predictive control implementation including the drone dynamics, solver and ROS2 wrapper. There are several ROS2 parameters that control the behaviour of the solver and drone.

| Parameter Name    | Brief Description     |
|--------------|--------------|
| Q | State weights |
| R  | Input Weights  |
|  initial_state 	 | Initial state of drone |
|  target_pos | Initial target state |
|  accel_max | Upper and lower bound for acceleration constraint |
|  N_horizon | Prediction Horizon for MPC |
|  prediction_period | Prediction Period for MPC |
|  drone_radius | Estimation of the drone's radius |
|  d_min | Spherical obstacle diameter |
|  noise | Include noise in the drone states |
|  noise_std_vel | Standard deviation for velocity noise |


### drone_simulator

Simulates the environment for the first scenario in pybullet.

### scenario_creator

Creates the more complex scenario needed for the second scenario. It includes classes to create obstacles of varying sizes and shapes, dynamic obstacles, markers and buildings. It simulates the drone and enviroment interaction and vizualizes the obstacles and paths. 

### Trajectory generator 

Creates waypoints for the drone to follow. The following trajectories are avaible:

| Trajectory Name    |
|--------------|
|Circular|
|Lemniscate|
|Linear|
|Spiral|
|Zigzag|
|Random|
