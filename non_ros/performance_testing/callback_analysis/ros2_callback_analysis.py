import sys
import argparse
import datetime as dt

from bokeh.plotting import figure, output_file, save
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, DatetimeTickFormatter, Spacer
import numpy as np
import pandas as pd

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

parser = argparse.ArgumentParser(description='Generate Bokeh callback duration report from ROS 2 trace.')
parser.add_argument('--trace_path', required=True, help='Path to ROS 2 trace directory')
parser.add_argument('--output_html', required=True, help='Output HTML file path')

args = parser.parse_args()

# Load and process trace
events = load_file(args.trace_path)
handler = Ros2Handler.process(events)
data_util = Ros2DataModelUtil(handler.data)
callback_symbols = data_util.get_callback_symbols()

# Output setup
output_file(args.output_html, title='Callback Analysis')
psize = 700
colours = ['#29788E', '#DD4968', '#410967']

plots = []
colour_i = 0

for obj, symbol in callback_symbols.items():
    owner_info = data_util.get_callback_owner_info(obj)
    if owner_info is None or '/parameter_events' in owner_info:
        continue

    duration_df = data_util.get_callback_durations(obj)
    if duration_df.empty:
        continue

    starttime = duration_df['timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
    source = ColumnDataSource(duration_df)

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

    plots.append(row(duration, Spacer(width=50), hist))

    colour_i = (colour_i + 1) % len(colours)

save(column(*plots))
