#!/usr/bin/env python3
"""
viewer_serialsend.py — Cockpit Viewer & Serial Telemetry Sender (Servo Telemetry & CSV Support)
"""
from __future__ import annotations
import argparse
import csv
import sys
import threading
import time
from collections import deque, defaultdict
from queue import Queue, Empty
from typing import List, Dict, Optional
import math
from pathlib import Path
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("pyserial is required. pip install pyserial", file=sys.stderr)
    raise

try:
    from pythonosc import udp_client
    OSC_AVAILABLE = True
except ImportError:
    OSC_AVAILABLE = False
    print("[WARN] pythonosc not available. OSC features disabled. Install with: pip install python-osc", file=sys.stderr)

try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False
    print("[WARN] pandas not available. CSV replay features disabled. Install with: pip install pandas", file=sys.stderr)

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Data Structure ---
class DataPoint:
    def __init__(self, t_ms, fields, src_seq):
        self.t_ms = t_ms
        self.fields = fields
        self.src_seq = src_seq

# --- OSC Client ---
class OSCSender:
    def __init__(self, ip, port):
        if not OSC_AVAILABLE:
            self.client = None
            return
        try:
            self.client = udp_client.SimpleUDPClient(ip, port)
            self.ip = ip
            self.port = port
            print(f"[OSC] Connected to {ip}:{port}")
        except Exception as e:
            print(f"[OSC] Failed to initialize: {e}")
            self.client = None
    
    def send_plane_data(self, roll, pitch, yaw, sv1=None, sv3=None):
        """Send plane data via OSC"""
        if not self.client:
            return
        try:
            data = [float(roll), float(pitch), float(yaw)]
            if sv1 is not None:
                data.append(float(sv1))
            if sv3 is not None:
                data.append(float(sv3))
            self.client.send_message("/plane/data", data)
        except Exception as e:
            print(f"[OSC] Send error: {e}")

# --- Serial Reader Thread ---
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue, save_csv=None, reconnect=True):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.reconnect = reconnect
        self._stop = threading.Event()
        self._ser = None
        self._csv_file = None
        self._csv_writer = None
        self._fields = []
        self._header_written = False

        if save_csv:
            # Append timestamp to filename for uniquely identifiable logs
            p = Path(save_csv)
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            unique_filename = f"{p.stem}_{timestamp}{p.suffix}"
            final_path = p.with_name(unique_filename)
            
            if final_path.parent and not final_path.parent.exists():
                final_path.parent.mkdir(parents=True, exist_ok=True)

            self._csv_file = open(str(final_path), "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            print(f"[LOG] Saving CSV to: {final_path}")

    def stop(self):
        self._stop.set()
        if self._ser: self._ser.close()
        if self._csv_file: self._csv_file.close()

    def write(self, data: bytes):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(data)
                print(f"[CMD] Sent: {data}")
            except Exception as e:
                print(f"[ERR] Write failed: {e}")

    def run(self):
        while not self._stop.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    self._ser = serial.Serial(self.port, self.baud, timeout=1)
                    print(f"[LOG] Connected to {self.port}")

                if self._ser.in_waiting > 0:
                    try:
                        line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                    except: continue
                    if not line: continue

                    if line.startswith("HDR,"):
                        self._parse_hdr(line)
                        self.out_queue.put(("HDR", line))
                    
                    elif line.startswith("DAT,"):
                        dp = self._parse_dat(line)
                        if dp:
                            self._log_csv(dp)
                            self.out_queue.put(("DAT", dp))

                    elif line.startswith("LOG,"):
                        self.out_queue.put(("LOG", line[4:]))
                    
                else:
                    time.sleep(0.005)

            except Exception as e:
                print(f"[ERR] Serial: {e}")
                if self._ser: self._ser.close(); self._ser = None
                if not self.reconnect: break
                time.sleep(2)

    def _parse_hdr(self, line):
        try:
            payload = line.split("fields=")[1]
            if ",rate=" in payload: payload = payload.split(",rate=")[0]
            new_fields = [x.strip() for x in payload.split(",")]
            
            # Update header information (CSV column definitions are also updated here)
            if self._fields != new_fields:
                self._fields = new_fields
                print(f"[HDR] Fields: {self._fields}")
                self._header_written = False 
        except: pass

    def _parse_dat(self, line):
        try:
            parts = line.split(",")
            if len(parts) < 3: return None
            seq = int(parts[1])
            t = float(parts[2])
            vals = [float(x) for x in parts[3:]]
            mapping = {}
            for i, name in enumerate(self._fields):
                if i < len(vals): mapping[name] = vals[i]
            return DataPoint(t, mapping, seq)
        except: return None

    def _log_csv(self, dp):
        # Write to CSV. If s0, s1, s2 are present in self._fields, they are recorded automatically.
        if self._csv_writer and self._fields:
            if not self._header_written:
                self._csv_writer.writerow(["src_seq", "t_ms"] + self._fields)
                self._header_written = True
            row = [dp.src_seq, dp.t_ms] + [dp.fields.get(f, "") for f in self._fields]
            self._csv_writer.writerow(row)

# --- CSV Replay to OSC ---
def replay_csv_to_osc(csv_file, ip, port, fps=50):
    """Replay CSV file data to OSC"""
    if not OSC_AVAILABLE:
        print("[ERR] OSC not available. Install python-osc")
        return False
    if not PANDAS_AVAILABLE:
        print("[ERR] Pandas not available. Install pandas")
        return False
    
    client = udp_client.SimpleUDPClient(ip, port)
    interval = 1.0 / fps
    
    # Try different encodings
    encodings = ['utf-8', 'cp932', 'utf-8-sig']
    df = None
    for enc in encodings:
        try:
            df = pd.read_csv(csv_file, encoding=enc, header=0)
            df.columns = df.columns.str.strip().str.lower()
            
            cols = ['roll', 'pitch', 'yaw', 'sv1', 'sv3']
            for col in cols:
                if col in df.columns:
                    df[col] = pd.to_numeric(df[col], errors='coerce')
            
            df = df.dropna(subset=['roll', 'pitch', 'yaw'])
            break
        except Exception as e:
            continue
    
    if df is None:
        print(f"[ERR] Failed to load CSV file: {csv_file}")
        return False
    
    print(f"[OSC] Starting replay: {len(df)} data points to {ip}:{port} at {fps} FPS")
    
    start_perf = time.perf_counter()
    try:
        for index, row in df.iterrows():
            target_time = start_perf + (index * interval)
            
            data = [
                float(row['roll']),
                float(row['pitch']),
                float(row['yaw'])
            ]
            
            # Add sv1 and sv3 if available
            if 'sv1' in row and pd.notna(row['sv1']):
                data.append(float(row['sv1']))
            if 'sv3' in row and pd.notna(row['sv3']):
                data.append(float(row['sv3']))
            
            client.send_message("/plane/data", data)
            
            wait_time = target_time - time.perf_counter()
            if wait_time > 0:
                time.sleep(wait_time)
        
        print("[OSC] Replay completed")
        return True
    except KeyboardInterrupt:
        print("\n[OSC] Replay stopped by user")
        return False
    except Exception as e:
        print(f"[OSC] Replay error: {e}")
        return False

# --- Main Viewer ---
def run_viewer(args):
    q = Queue()
    reader = SerialReader(args.port, args.baud, q, args.save)
    reader.start()
    
    # Initialize OSC sender if enabled
    osc_sender = None
    if args.osc_ip and args.osc_port:
        osc_sender = OSCSender(args.osc_ip, args.osc_port)

    plt.style.use('dark_background')
    fig, ax1 = plt.subplots(figsize=(12, 7))
    plt.subplots_adjust(bottom=0.25)
    
    ax2 = ax1.twinx()

    ax1.set_title(f"Flight Monitor (Dual Axis): {args.port}", fontsize=14, color='white')
    ax1.grid(True, linestyle="--", alpha=0.3)
    ax1.set_ylabel("Angle / Control", color='white')
    ax2.set_ylabel("Sensor / Accel", color='yellow')
    
    max_pts = args.max_points
    times = deque(maxlen=max_pts)
    series = defaultdict(lambda: deque(maxlen=max_pts))
    
    plot_lines = {}
    user_plot_fields = [x.strip() for x in args.plot.split(",")] if args.plot else []
    target_fields = []
    
    SENSOR_FIELDS = {'ax', 'ay', 'az', 'gx', 'gy', 'gz'}

    t0 = None

    mode_box = ax1.text(0.02, 0.95, "WAITING...", transform=ax1.transAxes, 
                       fontsize=16, fontweight='bold', color='yellow',
                       bbox=dict(facecolor='black', alpha=0.7, edgecolor='gray'))
    
    param_text = plt.figtext(0.02, 0.02, "Parameters: Waiting...", 
                             fontsize=11, family='monospace', color='cyan',
                             bbox=dict(facecolor='#111', alpha=0.8, edgecolor='cyan'))

    help_str = (
        "CONTROLS:\n[A]uto [M]anual [C]enter\n"
        "Pitch:[p/P][d/D][i/I] Roll:[x/X][y/Y][z/Z]\n"
        "Trim:[l/L][r/R][e/E] Goal:[g/G]\n"
        "Save:[s/S] Start:[h/H]"
    )
    plt.figtext(0.75, 0.02, help_str, fontsize=9, color='gray', family='monospace')

    def update(_):
        nonlocal t0, target_fields
        need_draw = False

        while True:
            try:
                kind, payload = q.get_nowait()
                
                if kind == "HDR":
                    f_str = payload.split("fields=")[1].split(",rate=")[0]
                    f_list = f_str.split(",")
                    
                    if user_plot_fields:
                         target_fields = [f for f in user_plot_fields if f in f_list]
                    else:
                         # Priority order of fields to display in plot
                         # Including servo angles (s0, s1, s2) displays them in the plot
                         target_fields = [f for f in ["roll", "pitch", "yaw", "s0", "s1", "s2", "ax", "ay", "az"] if f in f_list]
                         if not target_fields: target_fields = f_list[:3]
                    
                    ax1.clear()
                    ax2.clear()
                    ax1.grid(True, linestyle="--", alpha=0.3)
                    plot_lines.clear()

                    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
                    
                    for i, f in enumerate(target_fields):
                        color = colors[i % len(colors)]
                        if f in SENSOR_FIELDS:
                            plot_lines[f], = ax2.plot([], [], label=f, linestyle=':', linewidth=1.5, color=color)
                        else:
                            plot_lines[f], = ax1.plot([], [], label=f, linestyle='-', linewidth=2.0, color=color)

                    lines1, labels1 = ax1.get_legend_handles_labels()
                    lines2, labels2 = ax2.get_legend_handles_labels()
                    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
                    
                    ax1.add_artist(mode_box)
                    ax1.set_ylabel("Angle / Servo (deg)", color='white')
                    ax2.set_ylabel("Accel (G)", color='white')

                elif kind == "DAT":
                    if t0 is None: t0 = payload.t_ms / 1000.0
                    t = payload.t_ms / 1000.0 - t0
                    times.append(t)
                    for k, v in payload.fields.items():
                        series[k].append(v)
                    need_draw = True
                    
                    # Send to OSC if enabled
                    if osc_sender and osc_sender.client:
                        roll = payload.fields.get('roll', 0)
                        pitch = payload.fields.get('pitch', 0)
                        yaw = payload.fields.get('yaw', 0)
                        sv1 = payload.fields.get('sv1') or payload.fields.get('s0')
                        sv3 = payload.fields.get('sv3') or payload.fields.get('s2')
                        osc_sender.send_plane_data(roll, pitch, yaw, sv1, sv3)

                elif kind == "LOG":
                    print(f"[MCU] {payload}")
                    if "mode:Auto" in payload:
                        mode_box.set_text("MODE: AUTO")
                        mode_box.set_color("#00FF00")
                    elif "mode:Manual" in payload:
                        mode_box.set_text("MODE: MANUAL")
                        mode_box.set_color("#00FFFF")
                    elif "Param:" in payload:
                        p_str = payload.replace("Param:", "").replace(",", "  ")
                        param_text.set_text(f"PARAMS:\n{p_str}")

            except Empty: break

        if need_draw and target_fields and times:
            t_curr = list(times)
            for f in target_fields:
                if f in series and len(series[f]) == len(t_curr):
                    plot_lines[f].set_data(t_curr, list(series[f]))
            
            if len(t_curr) > 1:
                t_min = max(0, t_curr[-1] - args.window)
                t_max = t_curr[-1] + 0.1
                ax1.set_xlim(t_min, t_max)
                ax2.set_xlim(t_min, t_max)
                
                vals1 = []
                for f in target_fields:
                    if f not in SENSOR_FIELDS: vals1.extend(list(series[f])[-30:])
                if vals1:
                    ymin, ymax = min(vals1), max(vals1)
                    ymin = min(ymin, -5); ymax = max(ymax, 5)
                    pad = (ymax - ymin) * 0.1
                    ax1.set_ylim(ymin - pad, ymax + pad)

                vals2 = []
                for f in target_fields:
                    if f in SENSOR_FIELDS: vals2.extend(list(series[f])[-30:])
                if vals2:
                    ymin, ymax = min(vals2), max(vals2)
                    ymin = min(ymin, -0.5); ymax = max(ymax, 0.5)
                    pad = (ymax - ymin) * 0.1
                    ax2.set_ylim(ymin - pad, ymax + pad)

    ani = FuncAnimation(fig, update, interval=50)

    def on_key(event):
        k = event.key
        cmd = None
        if k == 'a': cmd = b'a'
        elif k == 'm': cmd = b'm'
        elif k == 'p': cmd = b'p'
        elif k == 'P': cmd = b'P'
        elif k == 'd': cmd = b'd'
        elif k == 'D': cmd = b'D'
        elif k == 'i': cmd = b'i'
        elif k == 'I': cmd = b'I'
        elif k == 'x': cmd = b'x'
        elif k == 'X': cmd = b'X'
        elif k == 'y': cmd = b'y'
        elif k == 'Y': cmd = b'Y'
        elif k == 'z': cmd = b'z'
        elif k == 'Z': cmd = b'Z'
        elif k == 'g': cmd = b'g'
        elif k == 'G': cmd = b'G'
        elif k == 'l': cmd = b'l'
        elif k == 'L': cmd = b'L'
        elif k == 'r': cmd = b'r'
        elif k == 'R': cmd = b'R'
        elif k == 'e': cmd = b'e'
        elif k == 'E': cmd = b'E'
        elif k == 'c' or k == 'C': cmd = b'c'
        elif k == 's' or k == 'S': cmd = b's'
        elif k == 'h' or k == 'H': cmd = b'h' # Handshake
        
        if cmd: reader.write(cmd)

    fig.canvas.mpl_connect('key_press_event', on_key)
    try: plt.show()
    except KeyboardInterrupt: pass
    finally: reader.stop()

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Flight Monitor with OSC support")
    p.add_argument("--port", type=str, help="Serial port (required for viewer mode)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--save", type=str, default=None, help="CSV file to save data")
    p.add_argument("--window", type=float, default=10.0)
    p.add_argument("--max-points", type=int, default=1000)
    p.add_argument("--plot", type=str, default=None)
    
    # OSC options
    p.add_argument("--osc-ip", type=str, default="10.227.81.102", help="OSC server IP address (default: 10.227.81.102)")
    p.add_argument("--osc-port", type=int, default=9000, help="OSC server port (default: 9000)")
    
    # CSV replay mode
    p.add_argument("--replay-csv", type=str, default=None, help="CSV file to replay to OSC")
    p.add_argument("--fps", type=int, default=50, help="FPS for CSV replay (default: 50)")
    
    args = p.parse_args()
    
    # CSV replay mode
    if args.replay_csv:
        if not args.osc_ip or not args.osc_port:
            print("[ERR] --osc-ip and --osc-port are required for CSV replay mode")
            sys.exit(1)
        replay_csv_to_osc(args.replay_csv, args.osc_ip, args.osc_port, args.fps)
    # Viewer mode
    elif args.port:
        run_viewer(args)
    else:
        print("[ERR] Either --port (for viewer) or --replay-csv (for replay) must be specified")
        p.print_help()
        sys.exit(1)