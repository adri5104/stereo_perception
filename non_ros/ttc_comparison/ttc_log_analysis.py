import argparse
import pandas as pd
from pathlib import Path
from bokeh.plotting import figure, output_file, save
from bokeh.layouts import column
from bokeh.models import ColumnDataSource, DatetimeTickFormatter

def main(input_file, output_dir):
    input_path = Path(input_file)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(input_path)

    # Convert timestamp to datetime
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

    # Handle inf values in min_ttc
    df['min_ttc'] = pd.to_numeric(df['min_ttc'], errors='coerce')
    df['min_ttc'] = df['min_ttc'].replace([float("inf"), float("-inf")], -50).fillna(-1)

    source = ColumnDataSource(df)

    # First plot: Min TTC
    p1 = figure(
        title="Minimum TTC over Time",
        x_axis_type="datetime",
        x_axis_label="Time",
        y_axis_label="Min TTC [s] (-50 = ∞)",
        width=800,
        height=300
    )
    p1.line(x='timestamp', y='min_ttc', source=source, line_width=2, color="#29788E")
    p1.xaxis.formatter = DatetimeTickFormatter(
        seconds="%H:%M:%S",
        minutes="%H:%M:%S",
        hours="%H:%M:%S"
    )
    p1.xaxis.major_label_orientation = 0.5

    # Second plot: Number of Objects
    p2 = figure(
        title="Number of Objects over Time",
        x_axis_type="datetime",
        x_axis_label="Time",
        y_axis_label="Detected Objects",
        width=800,
        height=300
    )
    p2.line(x='timestamp', y='num_objects', source=source, line_width=2, color="#DD4968")
    p2.xaxis.formatter = DatetimeTickFormatter(
        seconds="%H:%M:%S",
        minutes="%H:%M:%S",
        hours="%H:%M:%S"
    )
    p2.xaxis.major_label_orientation = 0.5

    output_file_path = output_path / (input_path.stem + ".html")
    output_file(str(output_file_path))
    save(column(p1, p2))

    print(f"✅ Plot saved at: {output_file_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot TTC CSV log as Bokeh HTML.")
    parser.add_argument("-i", "--input", required=True, help="Path to input CSV file")
    parser.add_argument("-o", "--output", required=True, help="Output directory for HTML plot")
    args = parser.parse_args()
    main(args.input, args.output)
