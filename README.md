# RO47005-Project

# Docker Instructions

See "./project_assets/docker/README_docker.md"

New packages and code goes into: ro47005_drone_simulator



# Installation of the Environment

## Using the RO47003 Singularity Container

Follow the instructions from the chapters 1 and 3 from this [manual](https://brightspace.tudelft.nl/d2l/le/content/682423/viewContent/4004834/View). Please download [this](https://surfdrive.surf.nl/files/index.php/s/PeR3mwCCXVv0xRT) singularity imgae instead the one linked in the manual.

After that, run these commands to install the [pybullet-drone-simulator](https://github.com/utiasDSL/gym-pybullet-drones) from wihtin the singularity terminal:

```
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones/

pip3 install -e . # if needed, `sudo apt install build-essential` to install `gcc` and build `pybullet`
```

To check the installation, run: 

```
cd gym_pybullet_drones/examples/
python3 pid.py # position and velocity reference
python3 pid_velocity.py # desired velocity reference
```

Download the project worspace by running:

```
git clone git@github.com:AntreasK24/RO47005-Project.git
cd drone_ws
colcon build
```

Check the workspace by running `ros2 run drone_simulator drone_simulator`.
