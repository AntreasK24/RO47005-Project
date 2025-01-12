### Build the container: 
sudo singularity build ros2_humble_acados_drones.sif path/to/file/singularity_RO47005_drone_ws_V01.def  | tee build_output.txt

##### For Jonah: 
sudo singularity build ~/RO47005/singularity/ros2_humble_acados_drones_03_mini.sif ~/RO47005/RO47005-Project/project_assets/singularity/singularity_RO47005_drone_ws_V03_mini.def  | tee ~/RO47005/singularity/build_output_03.txt


singularity shell --nv ~/RO47005/singularity/ros2_humble_acados_drones_02_mini.sif

### Run the container: singularity shell --nv path/to/file/ros2_humble_acados_drones.sif 

### After that:
source /opt/ros/humble/setup.bash

and to use gym_pybullet_drones OR acados, activate their virtual python environment:

source /gym_pybullet_drones/drone_env/bin/activate

OR 

source /acados/acados_env/bin/activate

type 'deactivate' to deactivate any of the virtual environments

If we want to scrape the virtual environments, we can change the buildfile of the container.




### Tests:

in drone_ws_:

colcon build 
ros2 run drone_mpc_python drone_mpc_python
ros2 launch scenario_creator scenario_launch.py
