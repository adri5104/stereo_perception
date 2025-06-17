import os
import time
import psutil
import pandas as pd
import matplotlib.pyplot as plt
import pynvml
from datetime import datetime
import threading
import argparse

from bokeh.plotting import figure, output_file, save
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, DatetimeTickFormatter, Span, Label, Spacer

# Sampling interval (in seconds)
interval = 0.08

# Nodes to track
tracked_nodes = ["visual_odometry", "kalman_filter_n", "optical_flow_co", "object_detector"]

# Initialize data structure
data = {node: {"timestamp": [], "cpu": [], "rss": [], "gpu": []} for node in tracked_nodes}

# Runtime flags
running = False
output_base_dir = ""


def find_node_processes():
    """Find the PIDs of the processes corresponding to the tracked ROS 2 nodes."""
    node_processes = {}
    for p in psutil.process_iter(['pid', 'cmdline']):
        try:
            cmd = " ".join(p.info['cmdline'])
            for node in tracked_nodes:
                if node in cmd:
                    node_processes[node] = p
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return node_processes


def get_gpu_usage(pid):
    """Return the GPU usage in MB for a specific process ID using pynvml."""
    try:
        for i in range(pynvml.nvmlDeviceGetCount()):
            handle = pynvml.nvmlDeviceGetHandleByIndex(i)
            procs = pynvml.nvmlDeviceGetComputeRunningProcesses(handle)
            for proc in procs:
                if proc.pid == pid:
                    return proc.usedGpuMemory / (1024 ** 2)
    except Exception:
        pass
    return 0.0


def monitor_processes():
    """Continuously monitor the resources used by each tracked process."""
    global running
    processes = find_node_processes()
    print("Monitoring the following nodes:", list(processes.keys()))

    while running:
        timestamp = datetime.now()
        for node, proc in processes.items():
            try:
                cpu = proc.cpu_percent(interval=None)
                mem = proc.memory_info().rss / (1024 ** 2)
                gpu = get_gpu_usage(proc.pid)
                data[node]["timestamp"].append(timestamp)
                data[node]["cpu"].append(cpu)
                data[node]["rss"].append(mem)
                data[node]["gpu"].append(gpu)
                print(f"[{timestamp:%H:%M:%S}] {node:20s} | CPU: {cpu:5.1f}% | MEM: {mem:6.1f} MB | GPU: {gpu:6.1f} MB")
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        time.sleep(interval)


def plot_matplotlib(output_dir):
    """Plot CPU, RSS, and GPU metrics using Matplotlib and save to PNG files."""
    for node in data:
        df = pd.DataFrame(data[node])
        if df.empty:
            continue
        df.set_index("timestamp", inplace=True)

        node_dir = os.path.join(output_dir, node)
        os.makedirs(node_dir, exist_ok=True)

        for metric, label in [("cpu", "CPU (%)"), ("rss", "Memory (MB)"), ("gpu", "GPU (MB)")]:
            fig, ax = plt.subplots(figsize=(10, 4))
            df[metric].plot(ax=ax, title=f"{node} - {label}")
            mean = df[metric].mean()
            std = df[metric].std()
            ax.axhline(mean, color='gray', linestyle='--', label=f"Mean: {mean:.2f}")
            ax.axhline(mean + std, color='red', linestyle=':', label=f"Std: +{std:.2f}")
            ax.axhline(max(mean - std, 0), color='blue', linestyle=':', label=f"Std: -{std:.2f}")
            ax.set_ylabel(label)
            ax.set_xlabel("Time")
            ax.legend()
            plt.grid(True)
            plt.tight_layout()
            file_path = os.path.join(node_dir, f"{metric}.png")
            plt.savefig(file_path)
            plt.close()
            print(f"Saved: {file_path}")


def plot_bokeh_html(output_dir):
    """Plot CPU, RSS, and GPU metrics using Bokeh and save as HTML."""
    plots = []
    colours = ['#29788E', '#DD4968', '#410967', '#3A3F58']
    colour_i = 0

    for node in data:
        df = pd.DataFrame(data[node])
        if df.empty:
            continue
        source = ColumnDataSource(df)
        ts = df['timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M:%S')

        row_plots = []
        for metric in ['cpu', 'rss', 'gpu']:
            plot = figure(
                title=f"{node} - {metric.upper()}",
                x_axis_label=f'Time ({ts})',
                y_axis_label= 'CPU (%)' if metric == 'cpu' else 'Memory (MB)' if metric == 'rss' else 'GPU (MB)',
                width=500, height=300,
            )
            plot.line(
                x='timestamp',
                y=metric,
                line_width=2,
                source=source,
                line_color=colours[colour_i % len(colours)],
            )
            plot.xaxis.formatter = DatetimeTickFormatter(seconds="%H:%M:%S")

            mean = df[metric].mean()
            std = df[metric].std()

            # Horizontal line at mean
            span = Span(location=mean, dimension='width', line_color="gray", line_dash='dashed', line_width=1)
            plot.add_layout(span)

            # Bottom-left corner annotation
            stats_text = f"μ = {mean:.2f}\nσ = {std:.2f}"
            label = Label(x=10, y=10, x_units='screen', y_units='screen',
                          text=stats_text,
                          text_font_size="8pt",
                          background_fill_color='white',
                          background_fill_alpha=0.6)
            plot.add_layout(label)

            row_plots.append(plot)
            colour_i += 1

        plots.append(row(*row_plots))

    output_path = os.path.join(output_dir, "resource_analysis.html")
    output_file(output_path, title="Resource Usage Analysis")
    save(column(*plots))
    print(f"Saved interactive HTML: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor resource usage of ROS 2 nodes")
    parser.add_argument("--output_dir", type=str, required=True, help="Base directory to store output analysis")
    args = parser.parse_args()

    # Init GPU support
    try:
        pynvml.nvmlInit()
    except:
        print("Warning: NVML could not be initialized, skipping GPU tracking.")
        get_gpu_usage = lambda pid: 0.0

    # Create timestamped output folder
    timestamp = datetime.now().strftime("analysis_%Y-%m-%d_%H-%M-%S")
    output_dir = os.path.join(args.output_dir, timestamp)
    os.makedirs(output_dir, exist_ok=True)

    # Start monitoring on ENTER
    input("Press ENTER to start monitoring...")
    running = True
    thread = threading.Thread(target=monitor_processes)
    thread.start()

    # Stop monitoring on ENTER
    input("Press ENTER again to stop and generate plots...")
    running = False
    thread.join()

    # Generate plots
    plot_matplotlib(output_dir)
    plot_bokeh_html(output_dir)

    print("Monitoring finished.")
