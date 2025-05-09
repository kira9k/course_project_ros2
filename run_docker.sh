#!/bin/bash

xhost +local:root

IMAGE_NAME="ros2_course_project"
HOST_WS="$(pwd)/" 
CONTAINER_WS="/root/ros2_ws" 

docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd):/root/ros2_ws" \
    $IMAGE_NAME \
    /bin/bash -c "rosdep install --from-paths src --ignore-src -r -y && colcon build --packages-select bcr_bot && /bin/bash -c 'source /root/ros2_ws/install/setup.bash && /bin/bash'"
