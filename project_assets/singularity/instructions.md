### Build the container: 
sudo singularity build ros2_humble_acados_drones.sif path/to/file/singularity_RO47005_drone_ws_V01.def  | tee build_output.txt

### Run the container: singularity shell --nv path/to/file/ros2_humble_acados_drones.sif 

### After that:
source /opt/ros/humble/setup.bash

and to use gym_pybullet_drones OR acados, activate their virtual python environment:

source /gym_pybullet_drones/drone_env/bin/activate

OR 

source /acados/acados_env/bin/activate

type 'deactivate' to deactivate any of the virtual environments

If we want to scrape the virtual environments, we can change the buildfile of the container.

