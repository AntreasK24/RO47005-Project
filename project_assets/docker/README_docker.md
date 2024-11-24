# RO47005-Project

# Docker Instructions

After cloning the repo 

``` 
cd  /docker/ros1-noetic-nongpu

#Build docker container 
sudo docker build -t ros_noetic .

#Run the image I reccomended creating an alias for this
sudo docker run -d \               
    --name="ros_noetic" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri \
    --volume="$HOME:/home/host_home:rw" \
    ros_noetic \
    tail -f /dev/null

#Enter the image
sudo docker exec -it ros_noetic zsh

```

Once inside the docker try running gazebo if it gives an error run the following on the host machine and try running gazebo again

```
xhost +local:docker
```

If this fixes the issue add the above command to your `~/.bashrc` file