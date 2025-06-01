# ROS 2 Callback Duration Analysis

This project provides a Python tool for analyzing the performance of ROS 2 nodes by visualizing the duration of their callbacks based on a recorded trace. It is useful for identifying performance bottlenecks and documenting timing behavior across nodes.

The tool processes a `ros2trace`-generated trace directory and outputs interactive HTML plots showing the execution duration and distribution of each callback.

## Installation and Setup

### 1. Install Required Tools

Make sure you have ROS 2 (e.g. Jazzy) installed. Then install the tracing utilities:

```bash
sudo apt-get update
sudo apt-get install -y \
  babeltrace \
  ros-jazzy-ros2trace \
  ros-jazzy-tracetools-analysis
```

Verify that tracing is enabled:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run tracetools status
# Should print: Tracing enabled
```

### 2. Install Python Dependencies

Create and activate a virtual environment (optional), then install dependencies:

```bash
pip install -r requirements.txt
```

The `requirements.txt` file should include:

```txt
bokeh
pandas
numpy
tracetools-analysis
```

## Recording a ROS 2 Trace

1. Start tracing in a terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 trace --session-name perf-test --list
```

Press **Enter** to begin recording.

2. In another terminal, launch your ROS 2 nodes or system under test.

3. When finished, stop the trace recording with **Ctrl+C** in the tracing terminal.

The trace will be stored in:

```
~/.ros/tracing/perf-test/
```

## Running the Analysis

Once a trace is recorded, analyze it by running:

```bash
python3 ./non_ros/performance_testing/ros2_callback_analysis.py
```

This script will:

- Load the trace from `~/.ros/tracing/perf-test/`
- Extract and process callback duration data
- Generate an interactive HTML file: `callback_analysis.html`

You can open the HTML file in a browser to explore the callback performance for each node.

## Output

The generated `callback_analysis.html` contains:

- Line plots showing callback durations over time
- Histograms showing duration distributions
- One row per callback group

This helps in identifying timing bottlenecks, jitter, or long-running callbacks.

## Running in Docker

Support for running this pipeline inside Docker is **work in progress**. Tracing and GUI-based output may require extra configuration for time sources, filesystem access, and display forwarding.


