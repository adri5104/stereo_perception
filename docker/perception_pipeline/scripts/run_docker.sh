#!/bin/bash
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../.. &> /dev/null && pwd )"
GITNAME="gitlab.lrz.de:5005/teleoperiertes_fahren/research_brecht/stereo_perception"
NAME="perception_pipeline"
TAG="1.1.0"
#-e CYCLONEDDS_URI=file:///home/ubuntu/ros2_ws/config/dds/autoware.xml \
docker run -it --rm \
    --name $NAME \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /dev/shm:/dev/shm \
    -v $PARENT_DIR/docker/$NAME/config/carnegie:/home/ubuntu/config \
    -v /home/edgar/david/adrian-ma/stereo_perception/config/dds/:/home/ubuntu/ros2_ws/config/dds/ \
    -v /sys/kernel/debug:/sys/kernel/debug \
    -w /home/ubuntu/ros2_ws \
    --network host \
    --cap-add=SYS_PTRACE \
    --cap-add=SYS_ADMIN \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/root/.Xauthority \
    --ipc=host \
    --pid=host \
    $GITNAME/$NAME:$TAG \
    "$@"