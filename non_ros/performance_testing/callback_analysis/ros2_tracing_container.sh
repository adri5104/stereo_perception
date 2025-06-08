#!/bin/bash

# Base path
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../.. &> /dev/null && pwd )"
DEST_BASE="$PARENT_DIR/non_ros/performance_testing/callback_analysis/docker"

show_help() {
    echo "Usage: $0 --container <name>"
    echo ""
    echo "  --container   Name of the Docker container (required)"
    echo ""
    echo "Example:"
    echo "  $0 --container perception_pipeline"
}

# Parse args
CONTAINER_NAME=""

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --container)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown parameter: $1"
            show_help
            exit 1
            ;;
    esac
done

if [[ -z "$CONTAINER_NAME" ]]; then
    echo "Error: --container is required."
    show_help
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^$CONTAINER_NAME\$"; then
    echo "Container '$CONTAINER_NAME' not running or doesn't exist."
    exit 1
fi

# Set up trace paths
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
TRACE_NAME="${TIMESTAMP}_trace"
CONTAINER_TRACE_DIR="/tmp/$TRACE_NAME"
HOST_TRACE_DIR="$DEST_BASE/$CONTAINER_NAME/$TRACE_NAME"

# Start tracing (user manually starts/ends)
# Start tracing interactively
echo "Starting trace in container '$CONTAINER_NAME'."
echo "  → Press Enter inside the container when prompted."
echo "  → Then press Ctrl+C when you want to stop tracing."

docker exec -it "$CONTAINER_NAME" bash -c "
  set -e
  mkdir -p \"$CONTAINER_TRACE_DIR\"
  bash -c 'source /opt/ros/humble/setup.bash && source /home/ubuntu/tracing_ws/install/setup.bash && ros2 trace -p \"$CONTAINER_TRACE_DIR\"'
"


# After Ctrl+C, continue...
echo ""
echo "Copying trace from container..."
mkdir -p "$HOST_TRACE_DIR"
docker cp "$CONTAINER_NAME:$CONTAINER_TRACE_DIR/." "$HOST_TRACE_DIR"
echo "Trace saved to: $HOST_TRACE_DIR"

# Call Python script to generate the HTML report
SESSION_DIR=$(find "$HOST_TRACE_DIR" -maxdepth 1 -type d -name "session-*")
TRACE_INPUT_DIR="$SESSION_DIR/ust/uid/1000/64-bit"
HTML_OUTPUT_PATH="$DEST_BASE/$CONTAINER_NAME/${TRACE_NAME}_callback_analysis.html"

python3 "$PARENT_DIR/non_ros/performance_testing/callback_analysis/ros2_callback_analysis.py" \
  --trace_path "$TRACE_INPUT_DIR" \
  --output_html "$HTML_OUTPUT_PATH"

# Clean up
rm -rf "$HOST_TRACE_DIR"
echo "Deleted trace directory from host: $HOST_TRACE_DIR"
