import sys

ROS_DISTRO = 'jazzy'
PYTHON_VERSION = '3.12'
sys.path.insert(0, f'/opt/ros/{ROS_DISTRO}/lib/{PYTHON_VERSION}/site-packages')
import datetime as dt

from bokeh.plotting import figure
from bokeh.plotting import output_file, save
from bokeh.layouts import column
from bokeh.io import show
from bokeh.layouts import row
from bokeh.models import ColumnDataSource
from bokeh.models import DatetimeTickFormatter
from bokeh.models import PrintfTickFormatter
import numpy as np
import pandas as pd
from bokeh.models import Spacer


from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

# Path to the trace file
path = '/home/adrian/.ros/tracing/perf-test'

# Path to the output file
output_file_path = '/home/adrian/stereo_perception/non_ros/performance_testing/callback_analysis.html'

# Process
events = load_file(path)
handler = Ros2Handler.process(events)

data_util = Ros2DataModelUtil(handler.data)

callback_symbols = data_util.get_callback_symbols()




output_file(output_file_path, title='Callback Analysis')
psize = 700
# If the trace contains more callbacks, add colours here
# or use: https://docs.bokeh.org/en/3.2.2/docs/reference/palettes.html
colours = ['#29788E', '#DD4968', '#410967']

plots = []  # list to store all callback plots

colour_i = 0  # index for cycling through colours
for obj, symbol in callback_symbols.items():
    owner_info = data_util.get_callback_owner_info(obj)
    if owner_info is None:
        owner_info = '[unknown]'

    # Skip internal ROS subscriptions
    if '/parameter_events' in owner_info:
        continue

    # Get callback duration data
    duration_df = data_util.get_callback_durations(obj)
    starttime = duration_df.loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
    source = ColumnDataSource(duration_df)

    # Line plot for callback durations
    duration = figure(
        title=owner_info,
        x_axis_label=f'start ({starttime})',
        y_axis_label='duration (ms)',
        width=psize, height=psize,
    )
    duration.title.align = 'center'
    duration.line(
        x='timestamp',
        y='duration',
        legend_label=str(symbol),
        line_width=2,
        source=source,
        line_color=colours[colour_i],
    )
    duration.legend.label_text_font_size = '11px'
    duration.xaxis[0].formatter = DatetimeTickFormatter(seconds='%Ss')

    # Histogram of durations (converted to ms)
    dur_hist, edges = np.histogram(duration_df['duration'] * 1000 / np.timedelta64(1, 's'))
    duration_hist = pd.DataFrame({
        'duration': dur_hist, 
        'left': edges[:-1], 
        'right': edges[1:],
    })
    hist = figure(
        title='Duration histogram',
        x_axis_label='duration (ms)',
        y_axis_label='frequency',
        width=psize, height=psize,
    )
    hist.title.align = 'center'
    hist.quad(
        bottom=0,
        top=duration_hist['duration'], 
        left=duration_hist['left'],
        right=duration_hist['right'],
        fill_color=colours[colour_i],
        line_color=colours[colour_i],
    )

    # Cycle through colors
    colour_i += 1
    colour_i %= len(colours)

    # Add the row of plots to the list
    spacer = Spacer(width=50)  # adjust width as needed
    plots.append(row(duration, spacer, hist))

save(column(*plots))  # save all callback plots to one HTML file

