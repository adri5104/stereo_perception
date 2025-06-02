#!/bin/bash
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../.. &> /dev/null && pwd )"
GITNAME="gitlab.lrz.de:5005/teleoperiertes_fahren/research_brecht/stereo_perception"
NAME="perception_pipeline"
TAG="1.0.2"

docker run -it --rm \
    --name $NAME \
    -v $(pwd):/ros2_ws \
    -w /home/ubuntu/perception_pipeline_ws \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    --cap-add=SYS_PTRACE \
    --cap-add=SYS_ADMIN \
    --volume /sys/kernel/debug:/sys/kernel/debug \
    --gpus all \
    -v /dev/shm:/dev/shm \
    -v $PARENT_DIR/docker/$NAME/config/carnegie:/home/ubuntu/config \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --ipc=host \
    --pid=host \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/root/.Xauthority \
    $GITNAME/$NAME:$TAG \
    "$@"