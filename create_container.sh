#!/bin/bash
set -e

# Variables
IMAGE_NAME=ros2_hmbl_img
CONTAINER_NAME=ros2_hmbl_container
DOCKERFILE_PATH=./Dockerfile  
WORKSPACE_PATH=./multirobot_ws
USER_NAME=test_user

# Build Docker image
echo "Building Docker image: $IMAGE_NAME"
docker build -t $IMAGE_NAME -f $DOCKERFILE_PATH .

# WSL setup
export DISPLAY=:0

echo "Starting container: $CONTAINER_NAME"
docker run -it --rm \
    --user $USER_NAME \
    --name=$CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$WORKSPACE_PATH:/home/$USER_NAME/multirobot_ws \
    --net=host \
    --privileged \
    $IMAGE_NAME \
    bash -c "
        source /opt/ros/humble/setup.bash
        cd /home/$USER_NAME/multirobot_ws
        colcon build
        echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
        echo 'source /home/$USER_NAME/multirobot_ws/install/setup.bash' >> ~/.bashrc
        
        source /opt/ros/humble/setup.bash
        source /home/$USER_NAME/multirobot_ws/install/setup.bash

        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/$USER_NAME/multirobot_ws/src/multirobot_sim/models:/opt/ros/humble/share/turtlebot3_gazebo/models

        ros2 launch multirobot_sim multirobot.launch.py 

        exec bash
    "

echo "Done."