#!/bin/sh

source ~/opt/ros/humble/setup.bash 
source install/local_setup.bash
#modify according to files location
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$HOME/acados/lib
