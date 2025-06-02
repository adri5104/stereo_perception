# Default values
#!/bin/bash
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../.. &> /dev/null && pwd )"
cd $PARENT_DIR

GITNAME="gitlab.lrz.de:5005/teleoperiertes_fahren/research_brecht/stereo_perception"
NAME="perception_pipeline"
TAG="1.0.2"
FILE=${PARENT_DIR}/docker/$NAME/Dockerfile

echo  $PARENT_DIR
echo $FILE

docker build --network host -f $FILE --tag $GITNAME/$NAME:$TAG  $PARENT_DIR  