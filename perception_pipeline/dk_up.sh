#!/bin/bash


dk_build_run() {
    docker run -it \
    --rm \
    --name ros2_perception_pipeline \
    -v $(pwd):/ros2_ws/src \
    --gpus all \
    --net=host \
    gitlab.lrz.de:5005/teleoperiertes_fahren/research_brecht/stereo_perception/perception_pipeline:dev
}

dk_build_run