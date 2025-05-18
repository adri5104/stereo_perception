# Default values
#!/bin/bash
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../.. &> /dev/null && pwd )"
cd $PARENT_DIR

GITNAME="gitlab.lrz.de:5005/teleoperiertes_fahren/research_brecht/stereo_perception"
NAME="stereo_perception"
TAG="1.0.0"

echo  $PARENT_DIR

docker build -f $PARENT_DIR/docker/Dockerfile --tag $GITNAME/$NAME:$TAG  $PARENT_DIR