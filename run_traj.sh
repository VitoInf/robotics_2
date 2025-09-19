#!/bin/bash
CONTAINER_NAME=ros2_hmbl_container
USER_NAME=test_user

# Execute ROS2 node in running container
docker exec -it $CONTAINER_NAME bash -c "
    source /opt/ros/humble/setup.bash
    source /home/$USER_NAME/multirobot_ws/install/setup.bash
    ros2 run multirobot_sim trajectory_generator
"