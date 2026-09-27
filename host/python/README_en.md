# Python Measurement & Telemetry Tools

[English](README_en.md) | [日本語](README.md)

A suite of Python host tools designed for receiving, visualizing, and logging telemetry data from the airframe and ground receivers, as well as sending real-time control commands to the flight controller. Optimized for experimental measurement workflows on Windows.

## Files

- `logger_gui.py`
  - Main measurement dashboard GUI built with `PySide6`. Integrates dual-channel serial reception, periodic camera JPEG capture with live preview, real-time multi-variable plotting, and numerical attitude display in a single window.
  - Data integrity and logging stability are the top priorities: CSV writing and image disk I/O are handled by dedicated background worker threads to guarantee zero data loss even under heavy GUI rendering loads.
- `main_logger.py`
  - Lightweight headless CLI logger. Operates without a GUI to record dual-channel serial data and camera snapshots directly to CSV and JPEG files.
- `live_plot_from_csv.py`
  - Simple live plotter for saved CSV files. Uses Matplotlib to tail and plot the latest incoming CSV log in real time with a configurable time window.
- `viewer_serialsend.py`
  - Real-time telemetry cockpit viewer & command transmitter. Connects to the ground ESP32 serial port to display attitude angles, acceleration, and servo deflections in real-time HUD-style plots, while supporting on-the-fly tuning of flight modes, PID gains, and trim angles via keyboard input, along with optional OSC streaming.

## Installation

Python 3.10 or higher is required.

```bash
pip install PySide6 pyqtgraph opencv-python pyserial numpy pandas matplotlib
```

Notes:
- `pandas` is used for post-processing and quick data loading.
- If you use the OSC streaming feature in `viewer_serialsend.py`, install `python-osc` (`pip install python-osc`).

## COM Port Check

List available COM ports:

```bash
python -m serial.tools.list_ports
```

Verify pyqtgraph installation and run examples:

```bash
python -m pyqtgraph.examples
```

## Running the Applications

Scripts can be executed directly from `host/python`:

### 1. GUI Measurement Dashboard (`logger_gui.py`)

```bash
python logger_gui.py
```

Basic Operating Steps:
1. Select `Serial1 Port` and `Serial2 Port`.
2. Select the `Camera`.
3. Adjust `Baudrate`, `Image Save Hz`, and `JPEG Quality` as needed.
   - `col_0`, `col_1`, and `col_2` from `serial1` are used as `roll`, `pitch`, and `yaw`.
4. Click **Start Logging**.
5. Click **Stop Logging** when the experiment finishes.

Attitude Display Specifications:
- `col_0`, `col_1`, and `col_2` of `serial1` are mapped to `roll`, `pitch`, and `yaw`.
- The GUI displays these as numerical values.

### 2. CLI Logger (`main_logger.py`)

For headless logging without a GUI:

```bash
python main_logger.py --serial1 COM3 --serial2 COM4 --baud 115200 --camera 0
```

### 3. CSV Live Plotter (`live_plot_from_csv.py`)

To follow and plot an active CSV log file in real time:

```bash
python live_plot_from_csv.py --column col_0 --window-ms 30000
```

### 4. Cockpit Viewer & Interactive Command Transmitter (`viewer_serialsend.py`)

To visualize incoming telemetry and interactively send tuning commands to the flight controller:

```bash
python viewer_serialsend.py --port COM3 --baud 115200 --save-csv telemetry.csv
```

- Send single-character commands via keyboard: `A` (Auto mode), `M` (Manual mode), `P`/`p` (Pitch PID gain), `L`/`l` (Left wing trim), etc.
- Forward data to external apps via OSC by specifying `--osc-ip 127.0.0.1 --osc-port 9000`.

## Saved Files

Each run creates a directory structure as follows:

```text
data/YYYYMMDD_HHMMSS/
  images/
    img_000000.jpg
    img_000001.jpg
  serial1.csv
  serial2.csv
  image_log.csv
  run_info.txt
```

`serial1.csv` / `serial2.csv`:

```text
pc_time_ms,pc_datetime,raw,col_0,col_1,...,col_19
```

`image_log.csv`:

```text
image_index,pc_time_ms,pc_datetime,filename
```

### Timestamp Specifications:

- `pc_time_ms`: Elapsed time in milliseconds from the start of the experiment.
- Reference clock: `time.perf_counter()`
- Saved with 3 decimal places (microsecond resolution).
- `pc_datetime`: ISO-formatted datetime string with milliseconds.

## Notes & Technical Details

- Default COM ports are `COM3` and `COM4`; default baudrate is `115200`.
- Serial input is parsed line-by-line; numeric values separated by commas, tabs, or whitespace are extracted and stored into `col_0..col_19`.
- Header lines are not parsed.
- Graph plots initially display `col_0` for `serial1` and `serial2`.
- Attitude display is strictly numerical (`roll`, `pitch`, `yaw`).
- Graphs display a rolling window of the most recent 30 seconds.
- Video files are not saved; instead, JPEG snapshots are captured and saved at the specified Hz.
- When stopping, worker threads are signaled to flush CSV buffers, thread join is awaited, and camera resources are released.
- Because data preservation is prioritized, GUI frame update rates may drop first in resource-constrained environments to avoid data loss.
