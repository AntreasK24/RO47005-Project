FROM osrf/ros:noetic-desktop-full

# Install additional packages for ROS, Gazebo, RViz, etc.
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-rviz \
    ros-noetic-rosbridge-suite \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers

# Install necessary packages
RUN apt-get update && \
    apt-get install -y x11-apps libgl1-mesa-glx libgl1-mesa-dri libegl1-mesa libglu1-mesa libx11-dev libxrandr-dev libxext-dev curl zsh git
    
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

    

# Set up the entrypoint to your ROS environment
ENTRYPOINT ["/ros_entrypoint.sh"]


# Setup environment variables for Gazebo
ENV QT_X11_NO_MITSHM=1

# Set ROS environment variables
SHELL ["/bin/bash", "-c"]

CMD ["/bin/zsh"]
