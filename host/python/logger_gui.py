#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import re
import argparse
import base64
import subprocess
import sys
import json
import threading
import time
from collections import deque
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from typing import Iterable

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pythonosc import udp_client
from PySide6.QtCore import QMutex, QObject, Qt, QThread, QTimer, Signal, Slot
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenuBar,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from shiboken6 import isValid

try:
    import cv2
except Exception as exc:
    print("opencv-python is required. Install with: pip install opencv-python", file=sys.stderr)
    raise

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:
    print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    raise


APP_TITLE = "Measurement Logger Dashboard"
WINDOW_SECONDS = 30.0
MAX_SERIAL_COLUMNS = 20
CSV_FLUSH_INTERVAL_SEC = 0.5
GRAPH_FPS = 20
PREVIEW_FPS = 15
RECORDING_PREVIEW_FPS = 5
SERIAL_UI_HZ = 20
SERIAL_READ_BUFFER_SIZE = 1024 * 1024
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
IMAGE_SAVE_HZ = 1.0
JPEG_QUALITY = 80
BAUDRATE = 115200
DEFAULT_SERIAL_PORTS = {"serial1": "COM3", "serial2": "COM4"}
OUT_ROOT = Path("data")
WINDOW_NAME = "Camera Preview"
NUMERIC_PATTERN = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
CAMERA_SCAN_MAX = 5
CAMERA_RELEASE_SETTLE_SEC = 0.3
CAMERA_BACKENDS = [
    (name, backend_id)
    for name, backend_id in (
        ("DSHOW", getattr(cv2, "CAP_DSHOW", None)),
        ("MSMF", getattr(cv2, "CAP_MSMF", None)),
    )
    if backend_id is not None
]
if not CAMERA_BACKENDS:
    CAMERA_BACKENDS = [("DEFAULT", None)]


def perf_ms(t0: float) -> float:
    return round((time.perf_counter() - t0) * 1000.0, 3)


def pc_datetime() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def parse_numeric_tokens(raw: str) -> list[object]:
    values: list[object] = []
    for token in NUMERIC_PATTERN.findall(raw)[:MAX_SERIAL_COLUMNS]:
        try:
            values.append(float(token))
        except ValueError:
            values.append(token)
    return values


def get_opencv_logger():
    utils = getattr(cv2, "utils", None)
    return getattr(utils, "logging", None) if utils is not None else None


@contextmanager
def suppress_opencv_warnings():
    logger = get_opencv_logger()
    if logger is None:
        yield
        return

    previous_level = logger.getLogLevel()
    logger.setLogLevel(logger.LOG_LEVEL_SILENT)
    try:
        yield
    finally:
        logger.setLogLevel(previous_level)


def iter_camera_backends(preferred_backend: str | None = None):
    if preferred_backend is None:
        yield from CAMERA_BACKENDS
        return

    yielded = set()
    for name, backend_id in CAMERA_BACKENDS:
        if name == preferred_backend:
            yielded.add(name)
            yield name, backend_id
    for name, backend_id in CAMERA_BACKENDS:
        if name not in yielded:
            yield name, backend_id


def try_open_camera(
    index: int,
    suppress_warnings: bool = False,
    preferred_backend: str | None = None,
) -> tuple[cv2.VideoCapture | None, str | None]:
    context = suppress_opencv_warnings() if suppress_warnings else nullcontext()
    with context:
        for backend_name, backend_id in iter_camera_backends(preferred_backend):
            cap = cv2.VideoCapture(index) if backend_id is None else cv2.VideoCapture(index, backend_id)
            if cap.isOpened():
                ok, _ = cap.read()
                if ok:
                    return cap, backend_name
            release_camera(cap)
    return None, None


def release_camera(cap: cv2.VideoCapture | None) -> None:
    if cap is None:
        return
    cap.release()
    time.sleep(CAMERA_RELEASE_SETTLE_SEC)


def open_camera_with_backend(
    index: int,
    backend_name: str | None,
    suppress_warnings: bool = False,
) -> cv2.VideoCapture | None:
    cap, _ = try_open_camera(
        index,
        suppress_warnings=suppress_warnings,
        preferred_backend=backend_name,
    )
    return cap


def stop_qthread(thread: QThread | None, timeout_ms: int = 3000) -> bool:
    if thread is None:
        return True
    try:
        if not isValid(thread):
            return True
        thread.quit()
        return thread.wait(timeout_ms)
    except RuntimeError:
        return True


def create_run_directory(out_root: Path) -> tuple[str, Path, Path]:
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = out_root / run_id
    images_dir = run_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    return run_id, run_dir, images_dir


def build_serial_header() -> list[str]:
    return ["pc_time_ms", "pc_datetime", "raw"] + [f"col_{index}" for index in range(MAX_SERIAL_COLUMNS)]


@dataclass
class SerialSample:
    serial_name: str
    pc_time_ms: float
    pc_datetime: str
    raw: str
    cols: list[object]


@dataclass
class LoggingConfig:
    serial1_port: str
    serial2_port: str
    baudrate: int
    camera_index: int
    camera_backend: str | None
    image_save_hz: float
    jpeg_quality: int
    camera_width: int = CAMERA_WIDTH
    camera_height: int = CAMERA_HEIGHT
    camera_fps: int = CAMERA_FPS
    serial_ports: list[tuple[str, int]] | None = None
    out_root: Path = OUT_ROOT


class SerialWorker(QObject):
    sample_ready = Signal(object)
    line_count_changed = Signal(str, int)
    status_changed = Signal(str)
    finished = Signal()
    error = Signal(str)

    def __init__(
        self,
        serial_name: str,
        port: str,
        baudrate: int,
        csv_path: Path | None,
        t0: float,
        stop_event: threading.Event,
        csv_t0: float | None = None,
        osc_client: udp_client.SimpleUDPClient | None = None,
        osc_path: str = "/plane/data",
        send_fps: int = 50,
    ) -> None:
        super().__init__()
        self.serial_name = serial_name
        self.port = port
        self.baudrate = baudrate
        self.csv_path = csv_path
        self.t0 = t0
        self.csv_t0 = csv_t0
        self.stop_event = stop_event
        self.osc_client = osc_client
        self.osc_path = osc_path
        self.send_fps = send_fps
        self._line_count = 0
        self._last_osc_send = 0.0
        self._osc_interval = 1.0 / send_fps if send_fps > 0 else 0.02
        self._last_ui_send = 0.0
        self._ui_interval = 1.0 / SERIAL_UI_HZ if SERIAL_UI_HZ > 0 else 0.05

    @Slot()
    def run(self) -> None:
        self.status_changed.emit(f"{self.serial_name}: opening {self.port}")
        try:
            file_ctx = self.csv_path.open("w", newline="", encoding="utf-8") if self.csv_path is not None else nullcontext()
            with file_ctx as csv_file:
                writer = csv.writer(csv_file) if csv_file is not None else None
                if writer is not None:
                    writer.writerow(build_serial_header())
                last_flush = time.perf_counter()

                with serial.Serial(self.port, self.baudrate, timeout=0.2) as ser:
                    try:
                        ser.set_buffer_size(rx_size=SERIAL_READ_BUFFER_SIZE)
                    except (AttributeError, serial.SerialException, OSError):
                        pass
                    self.status_changed.emit(f"{self.serial_name}: connected to {self.port}")
                    while not self.stop_event.is_set():
                        try:
                            raw_bytes = ser.readline()
                        except serial.SerialException as exc:
                            self.error.emit(f"{self.serial_name}: serial read failed: {exc}")
                            break

                        if not raw_bytes:
                            continue

                        raw = raw_bytes.decode("utf-8", errors="replace").strip()
                        if not raw:
                            continue

                        sample = SerialSample(
                            serial_name=self.serial_name,
                            pc_time_ms=perf_ms(self.t0),
                            pc_datetime=pc_datetime(),
                            raw=raw,
                            cols=parse_numeric_tokens(raw),
                        )

                        if writer is not None:
                            csv_time_ms = perf_ms(self.csv_t0) if self.csv_t0 is not None else sample.pc_time_ms
                            row = [f"{csv_time_ms:.3f}", sample.pc_datetime, sample.raw]
                            padded = list(sample.cols[:MAX_SERIAL_COLUMNS])
                            padded.extend([""] * (MAX_SERIAL_COLUMNS - len(padded)))
                            writer.writerow(row + padded)

                            now = time.perf_counter()
                            if now - last_flush >= CSV_FLUSH_INTERVAL_SEC:
                                csv_file.flush()
                                last_flush = now

                        self._line_count += 1
                        now = time.perf_counter()
                        if now - self._last_ui_send >= self._ui_interval:
                            self.line_count_changed.emit(self.serial_name, self._line_count)
                            self.sample_ready.emit(sample)
                            self._last_ui_send = now

                        if self.osc_client is not None and sample.serial_name == "serial1":
                            if now - self._last_osc_send >= self._osc_interval:
                                try:
                                    if len(sample.cols) >= 5:
                                        roll = float(sample.cols[0]) if isinstance(sample.cols[0], (float, int)) else 0.0
                                        pitch = float(sample.cols[1]) if isinstance(sample.cols[1], (float, int)) else 0.0
                                        yaw = float(sample.cols[2]) if isinstance(sample.cols[2], (float, int)) else 0.0
                                        sv1 = float(sample.cols[3]) if isinstance(sample.cols[3], (float, int)) else 0.0
                                        sv3 = float(sample.cols[4]) if isinstance(sample.cols[4], (float, int)) else 0.0
                                        self.osc_client.send_message(self.osc_path, [roll, pitch, yaw, sv1, sv3])
                                        self._last_osc_send = now
                                except Exception:
                                    pass

                if csv_file is not None:
                    csv_file.flush()
        except serial.SerialException as exc:
            self.error.emit(f"{self.serial_name}: could not open {self.port}: {exc}")
        except Exception as exc:
            self.error.emit(f"{self.serial_name}: unexpected error: {exc}")
        finally:
            if self._line_count > 0:
                self.line_count_changed.emit(self.serial_name, self._line_count)
            self.status_changed.emit(f"{self.serial_name}: stopped")
            self.finished.emit()


class CameraWorker(QObject):
    preview_ready = Signal(QImage)
    image_saved = Signal(int)
    image_recorded = Signal(float, str, str, int)
    status_changed = Signal(str)
    finished = Signal()
    error = Signal(str)

    def __init__(
        self,
        camera_index: int,
        camera_backend: str | None,
        t0: float,
        stop_event: threading.Event,
        image_log_path: Path,
        images_dir: Path,
        image_save_hz: float,
        jpeg_quality: int,
        camera_width: int,
        camera_height: int,
        camera_fps: int,
    ) -> None:
        super().__init__()
        self.camera_index = camera_index
        self.camera_backend = camera_backend
        self.t0 = t0
        self.stop_event = stop_event
        self.image_log_path = image_log_path
        self.images_dir = images_dir
        self.image_save_hz = max(image_save_hz, 0.01)
        self.jpeg_quality = jpeg_quality
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps

    @Slot()
    def run(self) -> None:
        process: subprocess.Popen[str] | None = None
        try:
            cmd = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--camera-capture-worker",
                "--camera-index",
                str(self.camera_index),
                "--image-log-path",
                str(self.image_log_path),
                "--images-dir",
                str(self.images_dir),
                "--image-save-hz",
                str(self.image_save_hz),
                "--jpeg-quality",
                str(self.jpeg_quality),
                "--camera-width",
                str(self.camera_width),
                "--camera-height",
                str(self.camera_height),
                "--camera-fps",
                str(self.camera_fps),
                "--t0",
                str(self.t0),
            ]
            if self.camera_backend:
                cmd.extend(["--camera-backend", self.camera_backend])

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )

            assert process.stdout is not None
            while not self.stop_event.is_set():
                line = process.stdout.readline()
                if not line:
                    if process.poll() is not None:
                        break
                    time.sleep(0.02)
                    continue

                line = line.strip()
                if line.startswith("STATUS "):
                    self.status_changed.emit(line[7:])
                elif line.startswith("SAVED "):
                    try:
                        self.image_saved.emit(int(line.split()[1]))
                    except (IndexError, ValueError):
                        pass
                elif line.startswith("ERROR "):
                    self.error.emit(line[6:])

            if self.stop_event.is_set() and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)

            return_code = process.poll()
            if return_code not in (0, None) and not self.stop_event.is_set():
                self.error.emit(f"camera process exited with code {return_code}")
        except Exception as exc:
            self.error.emit(f"camera: {exc}")
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
            self.status_changed.emit("camera: stopped")
            self.finished.emit()


class CameraPreviewWorker(QObject):
    preview_ready = Signal(QImage)
    image_saved = Signal(int)
    image_saved_at = Signal(int, float)
    status_changed = Signal(str)
    finished = Signal()
    error = Signal(str)

    def __init__(
        self,
        camera_index: int,
        camera_backend: str | None,
        stop_event: threading.Event,
        camera_width: int,
        camera_height: int,
        camera_fps: int,
    ) -> None:
        super().__init__()
        self.camera_index = camera_index
        self.camera_backend = camera_backend
        self.stop_event = stop_event
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps
        self.process: subprocess.Popen[str] | None = None
        self.pending_start_command: dict | None = None

    @Slot(str, str, str, float, int, float)
    def start_recording(
        self,
        image_log_path: str,
        images_dir: str,
        camera_backend: str,
        image_save_hz: float,
        jpeg_quality: int,
        t0: float,
    ) -> None:
        command = {
            "cmd": "START",
            "image_log_path": image_log_path,
            "images_dir": images_dir,
            "camera_backend": camera_backend or None,
            "image_save_hz": image_save_hz,
            "jpeg_quality": jpeg_quality,
            "t0": t0,
        }
        if self.process is None or self.process.stdin is None or self.process.poll() is not None:
            self.pending_start_command = command
            self.status_changed.emit("camera: preview is starting, recording will begin when ready")
            return
        self._write_command(command)

    def _write_command(self, command: dict) -> None:
        try:
            self.process.stdin.write(json.dumps(command) + "\n")
            self.process.stdin.flush()
        except Exception as exc:
            self.error.emit(f"camera: failed to start recording: {exc}")

    @Slot()
    def stop_recording(self) -> None:
        self.pending_start_command = None
        if self.process is None or self.process.stdin is None or self.process.poll() is not None:
            return
        try:
            self.process.stdin.write(json.dumps({"cmd": "STOP"}) + "\n")
            self.process.stdin.flush()
        except Exception:
            pass

    @Slot()
    def run(self) -> None:
        process = None
        try:
            cmd = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--camera-service-worker",
                "--camera-index",
                str(self.camera_index),
                "--camera-width",
                str(self.camera_width),
                "--camera-height",
                str(self.camera_height),
                "--camera-fps",
                str(self.camera_fps),
            ]
            if self.camera_backend:
                cmd.extend(["--camera-backend", self.camera_backend])

            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            self.process = process
            assert process.stdout is not None
            if self.pending_start_command is not None:
                self._write_command(self.pending_start_command)
                self.pending_start_command = None

            while not self.stop_event.is_set():
                line = process.stdout.readline()
                if not line:
                    if process.poll() is not None:
                        break
                    time.sleep(0.02)
                    continue

                line = line.strip()
                if line.startswith("PREVIEW "):
                    data = base64.b64decode(line[8:])
                    image = QImage.fromData(data, "JPG")
                    if not image.isNull():
                        self.preview_ready.emit(image)
                elif line.startswith("SAVED "):
                    try:
                        parts = line.split()
                        count = int(parts[1])
                        self.image_saved.emit(count)
                        if len(parts) >= 3:
                            self.image_saved_at.emit(count, float(parts[2]))
                    except (IndexError, ValueError):
                        pass
                elif line.startswith("STATUS "):
                    self.status_changed.emit(line[7:])
                elif line.startswith("ERROR "):
                    self.error.emit(line[6:])

            if process.poll() is None:
                if process.stdin is not None:
                    try:
                        process.stdin.write(json.dumps({"cmd": "EXIT"}) + "\n")
                        process.stdin.flush()
                    except Exception:
                        pass
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        except Exception as exc:
            self.error.emit(f"preview: {exc}")
        finally:
            self.process = None
            self.finished.emit()


class ReplayWorker(QObject):
    sample_ready = Signal(object)
    progress_updated = Signal(int, int)
    status_changed = Signal(str)
    finished = Signal()
    error = Signal(str)

    def __init__(
        self,
        csv_path: Path,
        original_fps: float,
        osc_client: udp_client.SimpleUDPClient | None = None,
        osc_path: str = "/plane/data",
        send_fps: int = 50,
        stop_event: threading.Event | None = None,
    ) -> None:
        super().__init__()
        self.csv_path = csv_path
        self.original_fps = max(original_fps, 1.0)
        self.osc_client = osc_client
        self.osc_path = osc_path
        self.send_fps = send_fps
        self.stop_event = stop_event or threading.Event()
        self.rows: list[list[str]] = []

    @Slot()
    def run(self) -> None:
        try:
            with self.csv_path.open("r", encoding="utf-8-sig") as csv_file:
                reader = csv.reader(csv_file)
                header = next(reader, None)
                if header is None:
                    self.error.emit("CSV file is empty")
                    return

                rows = list(reader)
                total = len(rows)
                if total == 0:
                    self.error.emit("No data rows in CSV")
                    return

                self.status_changed.emit(f"Replaying {total} samples")

                frame_interval = 1.0 / self.original_fps
                last_time = 0.0
                start_time = time.perf_counter()

                for idx, row in enumerate(rows):
                    if self.stop_event.is_set():
                        break

                    try:
                        pc_time_ms = float(row[0]) if len(row) > 0 else 0.0
                    except (ValueError, IndexError):
                        continue

                    if idx > 0:
                        try:
                            time_delta = (pc_time_ms - last_time) / 1000.0
                            sleep_time = time_delta - (time.perf_counter() - start_time)
                            if sleep_time > 0:
                                time.sleep(sleep_time)
                        except Exception:
                            pass

                    last_time = pc_time_ms
                    sample = SerialSample(
                        serial_name="replay",
                        pc_time_ms=pc_time_ms,
                        pc_datetime=row[1] if len(row) > 1 else "",
                        raw=row[2] if len(row) > 2 else "",
                        cols=[self._safe_float(row[i]) for i in range(3, min(len(row), 3 + MAX_SERIAL_COLUMNS))],
                    )
                    self.sample_ready.emit(sample)

                    if self.osc_client is not None and len(sample.cols) >= 5:
                        try:
                            roll = float(sample.cols[0]) if isinstance(sample.cols[0], (float, int)) else 0.0
                            pitch = float(sample.cols[1]) if isinstance(sample.cols[1], (float, int)) else 0.0
                            yaw = float(sample.cols[2]) if isinstance(sample.cols[2], (float, int)) else 0.0
                            sv1 = float(sample.cols[3]) if isinstance(sample.cols[3], (float, int)) else 0.0
                            sv3 = float(sample.cols[4]) if isinstance(sample.cols[4], (float, int)) else 0.0
                            self.osc_client.send_message(self.osc_path, [roll, pitch, yaw, sv1, sv3])
                        except Exception:
                            pass

                    self.progress_updated.emit(idx + 1, total)

        except Exception as exc:
            self.error.emit(f"Replay error: {exc}")
        finally:
            self.status_changed.emit("Replay finished")
            self.finished.emit()

    def _safe_float(self, value: str | object) -> object:
        if isinstance(value, (float, int)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value)
            except ValueError:
                return value
        return value


class SimpleOpenGLWidget(QWidget):
    """Model viewer placeholder (Cluster focus)"""
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 320)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        label = QLabel("3D Model Viewer\n(3D visualization disabled)\n\nFocus: Cluster OSC Communication")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet("""
            QLabel {
                background-color: #0f172a;
                color: #94a3b8;
                font-size: 12px;
                padding: 20px;
            }
        """)
        layout.addWidget(label)


class GraphSettingsDialog(QDialog):
    def __init__(self, parent=None, available_serials: list[str] | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Graph Settings")
        self.setModal(True)
        self.resize(400, 400)

        if available_serials is None:
            available_serials = ["serial1", "serial2"]

        layout = QVBoxLayout()

        self.checkboxes: dict[str, dict] = {}

        group = QGroupBox("Display Data")
        group_layout = QVBoxLayout()

        for serial_name in available_serials:
            checkbox = QCheckBox(serial_name)
            checkbox.setChecked(serial_name == "serial1")
            group_layout.addWidget(checkbox)
            self.checkboxes[serial_name] = {"checkbox": checkbox}

        group_layout.addStretch()
        group.setLayout(group_layout)
        layout.addWidget(group)

        layout_group = QGroupBox("Layout")
        layout_layout = QVBoxLayout()
        self.columns_spin = QSpinBox()
        self.columns_spin.setRange(1, 4)
        self.columns_spin.setValue(1)
        layout_layout.addWidget(QLabel("Graphs per row:"))
        layout_layout.addWidget(self.columns_spin)
        layout_group.setLayout(layout_layout)
        layout.addWidget(layout_group)

        button_layout = QHBoxLayout()
        ok_button = QPushButton("OK")
        cancel_button = QPushButton("Cancel")
        ok_button.clicked.connect(self.accept)
        cancel_button.clicked.connect(self.reject)
        button_layout.addStretch()
        button_layout.addWidget(ok_button)
        button_layout.addWidget(cancel_button)
        layout.addLayout(button_layout)

        self.setLayout(layout)

    def get_selected_serials(self) -> list[str]:
        return [name for name, data in self.checkboxes.items() if data["checkbox"].isChecked()]

    def get_columns(self) -> int:
        return self.columns_spin.value()


class ConfigurationDialog(QDialog):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.setModal(True)
        self.resize(500, 800)
        self.parent_window = parent

        main_layout = QVBoxLayout()

        tabs = QTabWidget()

        self.serial_group = QGroupBox("Serial Ports")
        self.serial_layout = QVBoxLayout(self.serial_group)
        self.serial_rows: list[dict] = []

        add_serial_btn = QPushButton("+ Add Serial Port")
        add_serial_btn.clicked.connect(self.add_serial_port)
        self.serial_layout.addWidget(add_serial_btn)
        self.serial_layout.addStretch()

        config_scroll = QWidget()
        config_layout = QVBoxLayout(config_scroll)
        config_layout.addWidget(self.serial_group)

        form_layout = QFormLayout()
        form_layout.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self.save_dir_edit = QLineEdit()
        self.save_dir_edit.setText(str(Path.home() / "measurements"))
        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.browse_save_dir)
        save_dir_row = QHBoxLayout()
        save_dir_row.addWidget(self.save_dir_edit, 1)
        save_dir_row.addWidget(self.browse_button)
        save_dir_widget = QWidget()
        save_dir_widget.setLayout(save_dir_row)

        self.camera_combo = QComboBox()
        self.refresh_cameras_button = QPushButton("Refresh Cameras")
        camera_row = QHBoxLayout()
        camera_row.addWidget(self.camera_combo)
        camera_row.addWidget(self.refresh_cameras_button)
        camera_row_widget = QWidget()
        camera_row_widget.setLayout(camera_row)

        self.image_hz_spin = QSpinBox()
        self.image_hz_spin.setRange(1, 30)
        self.image_hz_spin.setValue(int(IMAGE_SAVE_HZ))
        self.jpeg_quality_spin = QSpinBox()
        self.jpeg_quality_spin.setRange(10, 100)
        self.jpeg_quality_spin.setValue(JPEG_QUALITY)

        form_layout.addRow("Save Directory", save_dir_widget)
        form_layout.addRow("Camera", camera_row_widget)
        form_layout.addRow("Image Save Hz", self.image_hz_spin)
        form_layout.addRow("JPEG Quality", self.jpeg_quality_spin)

        config_layout.addLayout(form_layout)
        config_layout.addStretch()
        tabs.addTab(config_scroll, "Configuration")

        # Cluster Settings Tab
        cluster_scroll = QWidget()
        cluster_layout = QVBoxLayout(cluster_scroll)

        cluster_group = QGroupBox("Cluster OSC Settings")
        cluster_group_layout = QFormLayout(cluster_group)
        cluster_group_layout.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self.cluster_enable_check = QCheckBox()
        self.cluster_enable_check.setChecked(False)
        cluster_group_layout.addRow("Enable Cluster", self.cluster_enable_check)

        self.cluster_ip_edit = QLineEdit()
        self.cluster_ip_edit.setText("127.0.0.1")
        cluster_group_layout.addRow("Cluster IP", self.cluster_ip_edit)

        self.cluster_port_spin = QSpinBox()
        self.cluster_port_spin.setRange(1024, 65535)
        self.cluster_port_spin.setValue(9000)
        cluster_group_layout.addRow("Cluster Port", self.cluster_port_spin)

        self.osc_path_edit = QLineEdit()
        self.osc_path_edit.setText("/plane/data")
        cluster_group_layout.addRow("OSC Message Path", self.osc_path_edit)

        self.send_fps_spin = QSpinBox()
        self.send_fps_spin.setRange(1, 1000)
        self.send_fps_spin.setValue(50)
        cluster_group_layout.addRow("Send FPS", self.send_fps_spin)

        cluster_group.setLayout(cluster_group_layout)
        cluster_layout.addWidget(cluster_group)
        cluster_layout.addStretch()
        tabs.addTab(cluster_scroll, "Cluster Settings")

        graph_scroll = QWidget()
        graph_layout = QVBoxLayout(graph_scroll)

        self.checkboxes: dict[str, dict] = {}
        graph_group = QGroupBox("Display Data")
        graph_group_layout = QVBoxLayout()
        for serial_name in ["serial1", "serial2"]:
            checkbox = QCheckBox(serial_name)
            checkbox.setChecked(serial_name == "serial1")
            graph_group_layout.addWidget(checkbox)
            self.checkboxes[serial_name] = {"checkbox": checkbox}
        graph_group_layout.addStretch()
        graph_group.setLayout(graph_group_layout)
        graph_layout.addWidget(graph_group)

        layout_group = QGroupBox("Layout")
        layout_group_layout = QVBoxLayout()
        self.columns_spin = QSpinBox()
        self.columns_spin.setRange(1, 4)
        self.columns_spin.setValue(1)
        layout_group_layout.addWidget(QLabel("Graphs per row:"))
        layout_group_layout.addWidget(self.columns_spin)
        layout_group.setLayout(layout_group_layout)
        graph_layout.addWidget(layout_group)

        graph_layout.addStretch()
        tabs.addTab(graph_scroll, "Graph Settings")

        tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #334155; }
            QTabBar::tab {
                background-color: #1e293b;
                color: #cbd5e1;
                padding: 8px 20px;
                border: 1px solid #334155;
                border-bottom: none;
            }
            QTabBar::tab:selected {
                background-color: #0f172a;
                color: #e0e7ff;
                border-bottom: 2px solid #38bdf8;
            }
            QTabBar::tab:hover {
                background-color: #334155;
            }
        """)

        main_layout.addWidget(tabs)

        button_layout = QHBoxLayout()
        ok_button = QPushButton("OK")
        cancel_button = QPushButton("Cancel")
        ok_button.clicked.connect(self.accept)
        cancel_button.clicked.connect(self.reject)
        button_layout.addStretch()
        button_layout.addWidget(ok_button)
        button_layout.addWidget(cancel_button)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        if parent is not None:
            self.refresh_cameras_button.clicked.connect(parent.refresh_ports)
            self.refresh_cameras_button.clicked.connect(parent.refresh_cameras)

    def add_serial_port(self) -> None:
        row_num = len(self.serial_rows) + 1
        row_widget = QWidget()
        row_layout = QHBoxLayout(row_widget)
        row_layout.setContentsMargins(0, 0, 0, 0)

        port_combo = QComboBox()
        baudrate_edit = QLineEdit(str(BAUDRATE))
        baudrate_edit.setMaximumWidth(80)

        refresh_btn = QPushButton("↻")
        refresh_btn.setMaximumWidth(40)
        refresh_btn.clicked.connect(self.refresh_ports_for_combo)

        delete_btn = QPushButton("✕")
        delete_btn.setMaximumWidth(40)
        delete_btn.clicked.connect(lambda: self.remove_serial_port(len(self.serial_rows) - 1))

        row_layout.addWidget(QLabel(f"Port {row_num}:"), 0)
        row_layout.addWidget(port_combo, 1)
        row_layout.addWidget(QLabel("Baud:"), 0)
        row_layout.addWidget(baudrate_edit, 0)
        row_layout.addWidget(refresh_btn, 0)
        row_layout.addWidget(delete_btn, 0)

        self.serial_rows.append({
            "widget": row_widget,
            "combo": port_combo,
            "baudrate": baudrate_edit,
        })

        insert_pos = len(self.serial_rows)
        self.serial_layout.insertWidget(insert_pos, row_widget)
        self.refresh_ports_for_combo()

    def remove_serial_port(self, index: int) -> None:
        if 0 <= index < len(self.serial_rows):
            row = self.serial_rows.pop(index)
            row["widget"].deleteLater()

    def refresh_ports_for_combo(self) -> None:
        if self.parent_window is None or self.parent_window.configuration_dialog is None:
            return
        self.parent_window.refresh_ports()

    def get_serial_configs(self) -> list[tuple[str, int]]:
        configs = []
        for row in self.serial_rows:
            port = row["combo"].currentText().strip()
            try:
                baudrate = int(row["baudrate"].text().strip())
            except ValueError:
                baudrate = BAUDRATE
            if port:
                configs.append((port, baudrate))
        return configs

    def set_serial_ports(self, ports: list[str]) -> None:
        for row in self.serial_rows:
            current = row["combo"].currentText()
            row["combo"].clear()
            row["combo"].addItems(ports)
            if current in ports:
                row["combo"].setCurrentText(current)

    def browse_save_dir(self) -> None:
        dir_path = QFileDialog.getExistingDirectory(self, "Select Save Directory")
        if dir_path:
            self.save_dir_edit.setText(dir_path)

    def get_out_root(self) -> Path:
        return Path(self.save_dir_edit.text().strip()) if self.save_dir_edit.text().strip() else Path.home() / "measurements"

    def save_config(self, config_file: Path) -> None:
        import json
        config_data = {
            "save_dir": self.save_dir_edit.text(),
            "camera": self.camera_combo.currentText(),
            "image_hz": self.image_hz_spin.value(),
            "jpeg_quality": self.jpeg_quality_spin.value(),
            "serial_ports": [
                {"port": row["combo"].currentText(), "baudrate": row["baudrate"].text()}
                for row in self.serial_rows
            ],
            "cluster_enabled": self.cluster_enable_check.isChecked(),
            "cluster_ip": self.cluster_ip_edit.text(),
            "cluster_port": self.cluster_port_spin.value(),
            "osc_path": self.osc_path_edit.text(),
            "send_fps": self.send_fps_spin.value(),
        }
        config_file.parent.mkdir(parents=True, exist_ok=True)
        config_file.write_text(json.dumps(config_data, indent=2), encoding="utf-8")

    def load_config(self, config_file: Path) -> None:
        if not config_file.exists():
            return
        try:
            import json
            config_data = json.loads(config_file.read_text(encoding="utf-8"))

            if "save_dir" in config_data:
                self.save_dir_edit.setText(config_data["save_dir"])

            if "camera" in config_data:
                self._select_combo_value(self.camera_combo, config_data["camera"])

            if "image_hz" in config_data:
                self.image_hz_spin.setValue(config_data["image_hz"])

            if "jpeg_quality" in config_data:
                self.jpeg_quality_spin.setValue(config_data["jpeg_quality"])

            if "serial_ports" in config_data:
                while len(self.serial_rows) > 0:
                    self.remove_serial_port(len(self.serial_rows) - 1)

                for port_config in config_data["serial_ports"]:
                    self.add_serial_port()
                    if self.serial_rows:
                        row = self.serial_rows[-1]
                        row["combo"].setCurrentText(port_config.get("port", ""))
                        row["baudrate"].setText(port_config.get("baudrate", str(BAUDRATE)))

            if "cluster_enabled" in config_data:
                self.cluster_enable_check.setChecked(config_data["cluster_enabled"])

            if "cluster_ip" in config_data:
                self.cluster_ip_edit.setText(config_data["cluster_ip"])

            if "cluster_port" in config_data:
                self.cluster_port_spin.setValue(config_data["cluster_port"])

            if "osc_path" in config_data:
                self.osc_path_edit.setText(config_data["osc_path"])

            if "send_fps" in config_data:
                self.send_fps_spin.setValue(config_data["send_fps"])
        except Exception:
            pass

    def _select_combo_value(self, combo: QComboBox, value: str) -> None:
        index = combo.findText(value)
        combo.setCurrentIndex(index if index >= 0 else 0)

    def get_selected_serials(self) -> list[str]:
        return [name for name, data in self.checkboxes.items() if data["checkbox"].isChecked()]

    def get_columns(self) -> int:
        return self.columns_spin.value()

    def get_cluster_settings(self) -> dict:
        return {
            "enabled": self.cluster_enable_check.isChecked(),
            "ip": self.cluster_ip_edit.text().strip(),
            "port": self.cluster_port_spin.value(),
            "osc_path": self.osc_path_edit.text().strip(),
            "send_fps": self.send_fps_spin.value(),
        }


class DashboardWindow(QMainWindow):
    camera_start_requested = Signal(str, str, str, float, int, float)
    camera_stop_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1600, 980)

        self.stop_event: threading.Event | None = None
        self.serial_threads: list[QThread] = []
        self.camera_thread: QThread | None = None
        self.preview_thread: QThread | None = None
        self.preview_worker: CameraPreviewWorker | None = None
        self.preview_stop_event: threading.Event | None = None
        self.preview_serial_stop_event: threading.Event | None = None
        self.preview_serial_threads: list[QThread] = []
        self.preview_serial_workers: list[QObject] = []
        self.workers: list[QObject] = []
        self.logging_active = False
        self.is_closing = False
        self.preview_started_once = False
        self.current_run_dir: Path | None = None
        self.current_images_dir: Path | None = None
        self.current_t0 = 0.0
        self.graph_t0 = time.perf_counter()
        self.latest_pose = (0.0, 0.0, 0.0)
        self.preview_mutex = QMutex()
        self.camera_backend_map: dict[int, str | None] = {}
        self.graph_queue: Queue[SerialSample] = Queue()
        self.series_data: dict[str, dict] = {
            "serial1": {"time": deque(maxlen=10000), "cols": {}},
            "serial2": {"time": deque(maxlen=10000), "cols": {}},
        }
        self.serial_curves: dict[str, dict[int, pg.PlotDataItem]] = {}
        self.recording_markers: list[dict[str, object]] = []
        self.recording_marker_items: dict[str, list[object]] = {}
        self.pending_recording_start_marker = False
        self.recording_graph_start_ms = 0.0
        self.last_image_graph_time_ms: float | None = None
        self.configuration_dialog: ConfigurationDialog | None = None
        self.countdown_active = False
        self.config_file = Path.home() / ".claude" / "logger_config.json"
        self.selected_graphs: list[str] = ["serial1"]
        self.graph_columns = 1
        self.serial_port_map: dict[str, str] = {}
        self.graphs_container: QWidget | None = None
        self.graphs_layout: QGridLayout | None = None
        self.plot_widgets: dict[str, pg.PlotWidget] = {}
        self.osc_client: udp_client.SimpleUDPClient | None = None
        self.replay_thread: QThread | None = None
        self.replay_worker: ReplayWorker | None = None
        self.replay_stop_event: threading.Event | None = None
        self.replaying = False
        self.selected_replay_folder: Path | None = None

        self._build_ui()
        self._apply_dark_theme()
        self.configuration_dialog.load_config(self.config_file)
        self.refresh_ports()
        self.refresh_cameras()
        self._init_timers()

    def _build_ui(self) -> None:
        menubar = QMenuBar(self)
        self.setMenuBar(menubar)
        menu = menubar.addMenu("☰ Menu")
        config_action = menu.addAction("Configuration")
        config_action.triggered.connect(self.open_configuration_dialog)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)

        title = QLabel("Measurement Logger Dashboard")
        title.setObjectName("titleLabel")
        root.addWidget(title)

        top_layout = QGridLayout()
        top_layout.setHorizontalSpacing(12)
        top_layout.setVerticalSpacing(12)
        root.addLayout(top_layout, 0)

        self.preview_label = QLabel("Camera preview")
        self.preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_label.setMinimumSize(480, 320)
        self.preview_label.setMaximumSize(480, 320)
        self.preview_label.setFrameShape(QFrame.Shape.StyledPanel)
        self.preview_label.setObjectName("previewLabel")
        top_layout.addWidget(self._wrap_group("Camera Preview", self.preview_label), 0, 0, 1, 1)

        self.graphs_container = QWidget()
        self.graphs_layout = QGridLayout(self.graphs_container)
        self.graphs_layout.setSpacing(12)
        self._build_graphs()
        top_layout.addWidget(self._wrap_group("Realtime Graphs", self.graphs_container), 0, 1, 1, 1)

        self.configuration_dialog = ConfigurationDialog(self)

        bottom_layout = QGridLayout()
        bottom_layout.setHorizontalSpacing(12)
        bottom_layout.setVerticalSpacing(12)
        root.addLayout(bottom_layout)

        control_group = QGroupBox("Logging Control")
        control_layout = QVBoxLayout(control_group)
        self.start_button = QPushButton("Start Logging")
        self.stop_button = QPushButton("Stop Logging")
        self.stop_button.setEnabled(False)
        self.recording_label = QLabel("IDLE")
        self.recording_label.setObjectName("recordingIdle")
        self.save_dir_label = QLabel("-")
        self.save_dir_label.setWordWrap(True)
        control_layout.addWidget(self.start_button)
        control_layout.addWidget(self.stop_button)
        control_layout.addWidget(QLabel("Recording State"))
        control_layout.addWidget(self.recording_label)
        control_layout.addWidget(QLabel("Save Directory"))
        control_layout.addWidget(self.save_dir_label)
        control_layout.addStretch(1)
        bottom_layout.addWidget(control_group, 0, 0, 1, 1)

        playback_group = QGroupBox("Playback Control")
        playback_layout = QVBoxLayout(playback_group)
        self.browse_replay_button = QPushButton("Browse Data Folder")
        self.browse_replay_button.clicked.connect(self.browse_replay_folder)
        self.replay_folder_label = QLabel("-")
        self.replay_folder_label.setWordWrap(True)
        self.replay_count_label = QLabel("Data points: 0")
        self.replay_date_label = QLabel("Date range: -")
        self.play_button = QPushButton("Play")
        self.play_button.setEnabled(False)
        self.stop_replay_button = QPushButton("Stop Playback")
        self.stop_replay_button.setEnabled(False)
        self.replay_progress_label = QLabel("0 / 0")
        playback_layout.addWidget(self.browse_replay_button)
        playback_layout.addWidget(QLabel("Selected Folder:"))
        playback_layout.addWidget(self.replay_folder_label)
        playback_layout.addWidget(self.replay_count_label)
        playback_layout.addWidget(self.replay_date_label)
        playback_layout.addWidget(self.play_button)
        playback_layout.addWidget(self.stop_replay_button)
        playback_layout.addWidget(QLabel("Progress:"))
        playback_layout.addWidget(self.replay_progress_label)
        playback_layout.addStretch(1)
        bottom_layout.addWidget(playback_group, 0, 1, 1, 1)

        stats_group = QGroupBox("Runtime Stats")
        stats_layout = QFormLayout(stats_group)
        self.serial1_count_label = QLabel("0")
        self.serial2_count_label = QLabel("0")
        self.image_count_label = QLabel("0")
        self.roll_label = QLabel("0.0 deg")
        self.pitch_label = QLabel("0.0 deg")
        self.yaw_label = QLabel("0.0 deg")
        stats_layout.addRow("serial1 lines", self.serial1_count_label)
        stats_layout.addRow("serial2 lines", self.serial2_count_label)
        stats_layout.addRow("saved images", self.image_count_label)
        stats_layout.addRow("roll", self.roll_label)
        stats_layout.addRow("pitch", self.pitch_label)
        stats_layout.addRow("yaw", self.yaw_label)
        bottom_layout.addWidget(stats_group, 0, 2, 1, 1)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        self.configuration_dialog.camera_combo.currentTextChanged.connect(self.on_camera_selection_changed)
        self.start_button.clicked.connect(self.start_logging)
        self.stop_button.clicked.connect(self.stop_logging)
        self.play_button.clicked.connect(self.start_replay)
        self.stop_replay_button.clicked.connect(self.stop_replay)

    def _wrap_group(self, title: str, widget: QWidget) -> QGroupBox:
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        layout.addWidget(widget)
        return group

    def _with_button(self, widget: QWidget, button: QPushButton | None) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(widget, 1)
        if button is not None:
            layout.addWidget(button)
        return container

    def _apply_dark_theme(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background-color: #0b1118;
                color: #e5eef7;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #243142;
                border-radius: 10px;
                margin-top: 14px;
                padding-top: 10px;
                background-color: #111926;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
                color: #93c5fd;
            }
            QPushButton {
                background-color: #1d4ed8;
                border: none;
                border-radius: 8px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #2563eb;
            }
            QPushButton:disabled {
                background-color: #334155;
                color: #94a3b8;
            }
            QLineEdit, QComboBox, QSpinBox {
                background-color: #0f172a;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 6px 8px;
            }
            QLabel#titleLabel {
                font-size: 24px;
                font-weight: 700;
                color: #f8fafc;
            }
            QLabel#previewLabel {
                background-color: #04070b;
                border: 1px solid #334155;
                border-radius: 8px;
            }
            QLabel#recordingIdle {
                color: #fbbf24;
                font-size: 20px;
                font-weight: 700;
            }
            QLabel#recordingActive {
                color: #f43f5e;
                font-size: 20px;
                font-weight: 700;
            }
            QLabel#poseSummary {
                background-color: #10151c;
                border: 1px solid #334155;
                border-radius: 8px;
                color: #dbeafe;
                font-size: 22px;
                font-weight: 600;
                line-height: 1.5;
            }
            """
        )

    def _init_timers(self) -> None:
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self.drain_graph_queue)
        self.graph_timer.start(int(1000 / GRAPH_FPS))

    @Slot()
    def open_configuration_dialog(self) -> None:
        if self.configuration_dialog is not None:
            if self.configuration_dialog.exec() == QDialog.DialogCode.Accepted:
                self.configuration_dialog.save_config(self.config_file)
                self.selected_graphs = self.configuration_dialog.get_selected_serials()
                self.graph_columns = self.configuration_dialog.get_columns()
                serial_configs = self.configuration_dialog.get_serial_configs()
                self.serial_port_map = {f"serial{i+1}": port for i, (port, _) in enumerate(serial_configs)}
                self._build_graphs()
                if not self.logging_active:
                    self.start_serial_preview()

    def _build_graphs(self) -> None:
        if self.graphs_layout is None:
            return

        while self.graphs_layout.count() > 0:
            item = self.graphs_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.plot_widgets.clear()
        self.serial_curves.clear()

        for idx, serial_name in enumerate(self.selected_graphs):
            plot = pg.PlotWidget()
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend()
            plot.setLabel("bottom", "time (ms)")
            plot.setLabel("left", "value")
            plot.setMouseEnabled(x=False, y=False)
            plot.setBackground("#121820")
            port_label = self.serial_port_map.get(serial_name, serial_name)
            plot.setTitle(port_label, color="#94a3b8", size="10pt")

            self.plot_widgets[serial_name] = plot
            self.serial_curves[serial_name] = {}
            self.recording_marker_items[serial_name] = []

            row = idx // self.graph_columns
            col = idx % self.graph_columns
            self.graphs_layout.addWidget(plot, row, col)
            self._refresh_recording_markers(serial_name)

    def _add_recording_marker(self, marker_type: str, time_ms: float) -> None:
        color = "#ef4444" if marker_type == "start" else "#f59e0b"
        label = "START" if marker_type == "start" else "STOP"
        self.recording_markers.append({"type": marker_type, "time_ms": time_ms})
        for serial_name in self.plot_widgets:
            self._add_recording_marker_to_plot(serial_name, marker_type, time_ms, color, label)

    def _add_recording_marker_to_plot(
        self,
        serial_name: str,
        marker_type: str,
        time_ms: float,
        color: str,
        label: str,
    ) -> None:
        plot = self.plot_widgets.get(serial_name)
        if plot is None:
            return
        line = pg.InfiniteLine(
            pos=time_ms,
            angle=90,
            movable=False,
            pen=pg.mkPen(color, width=2, style=Qt.PenStyle.DashLine),
        )
        text = pg.TextItem(label, color=color, anchor=(0.0, 1.0))
        text.setPos(time_ms, 0)
        plot.addItem(line)
        plot.addItem(text)
        self.recording_marker_items.setdefault(serial_name, []).extend([line, text])

    def _refresh_recording_markers(self, serial_name: str) -> None:
        self._clear_recording_marker_items(serial_name)
        for marker in self.recording_markers:
            marker_type = str(marker["type"])
            color = "#ef4444" if marker_type == "start" else "#f59e0b"
            label = "START" if marker_type == "start" else "STOP"
            self._add_recording_marker_to_plot(serial_name, marker_type, float(marker["time_ms"]), color, label)

    def _clear_recording_marker_items(self, serial_name: str | None = None) -> None:
        names = [serial_name] if serial_name is not None else list(self.recording_marker_items)
        for name in names:
            plot = self.plot_widgets.get(name)
            for item in self.recording_marker_items.get(name, []):
                if plot is not None:
                    try:
                        plot.removeItem(item)
                    except Exception:
                        pass
            self.recording_marker_items[name] = []

    def _show_preview_dialog(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle("Camera Preview")
        dialog.setModal(True)
        dialog.resize(640, 480)

        layout = QVBoxLayout()
        preview_label = QLabel()
        preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        preview_label.setMinimumSize(640, 480)
        layout.addWidget(preview_label)

        button_layout = QHBoxLayout()
        ok_button = QPushButton("OK")
        ok_button.clicked.connect(dialog.accept)
        button_layout.addStretch()
        button_layout.addWidget(ok_button)
        layout.addLayout(button_layout)

        dialog.setLayout(layout)

        try:
            config = self.collect_config()
            cap = cv2.VideoCapture(config.camera_index)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = frame.shape
                    bytes_per_line = ch * w
                    qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                    pixmap = QPixmap.fromImage(qt_image).scaledToWidth(640, Qt.TransformationMode.SmoothTransformation)
                    preview_label.setPixmap(pixmap)
                release_camera(cap)
        except Exception as exc:
            preview_label.setText(f"Preview Error:\n{exc}")

        dialog.exec()

    def _show_countdown(self) -> None:
        dialog = QMessageBox(self)
        dialog.setWindowTitle("Recording Starts In...")
        dialog.setIcon(QMessageBox.Icon.Information)
        dialog.setStandardButtons(QMessageBox.StandardButton.NoButton)

        for count in [3, 2, 1]:
            dialog.setText(str(count))
            dialog.show()
            QApplication.processEvents()
            time.sleep(1)

        dialog.close()

    @Slot()
    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        if not ports:
            ports = [DEFAULT_SERIAL_PORTS["serial1"], DEFAULT_SERIAL_PORTS["serial2"]]
        self.configuration_dialog.set_serial_ports(ports)

    @Slot()
    def refresh_cameras(self) -> None:
        current = self.configuration_dialog.camera_combo.currentText()
        cameras = [str(index) for index in range(CAMERA_SCAN_MAX)]
        self.camera_backend_map = {}
        self.configuration_dialog.camera_combo.clear()
        self.configuration_dialog.camera_combo.addItems(cameras)
        self._select_combo_value(self.configuration_dialog.camera_combo, current or "0")
        if not self.logging_active and self.preview_started_once:
            self.start_idle_preview()

    def _select_combo_value(self, combo: QComboBox, value: str) -> None:
        index = combo.findText(value)
        combo.setCurrentIndex(index if index >= 0 else 0)

    def collect_config(self) -> LoggingConfig:
        camera_index = int(self.configuration_dialog.camera_combo.currentText())
        serial_configs = self.configuration_dialog.get_serial_configs()

        serial1_port = ""
        serial2_port = ""
        if len(serial_configs) > 0:
            serial1_port = serial_configs[0][0]
        if len(serial_configs) > 1:
            serial2_port = serial_configs[1][0]

        return LoggingConfig(
            serial1_port=serial1_port,
            serial2_port=serial2_port,
            baudrate=BAUDRATE,
            camera_index=camera_index,
            camera_backend=self.camera_backend_map.get(camera_index),
            image_save_hz=float(self.configuration_dialog.image_hz_spin.value()),
            jpeg_quality=int(self.configuration_dialog.jpeg_quality_spin.value()),
            serial_ports=serial_configs,
            out_root=self.configuration_dialog.get_out_root(),
        )

    @Slot()
    def start_logging(self) -> None:
        if self.logging_active:
            return

        if not self.stop_serial_preview():
            QMessageBox.warning(self, "Please Retry", "Serial preview is still stopping. Please press Start Logging again.")
            return

        try:
            config = self.collect_config()
        except Exception as exc:
            QMessageBox.critical(self, "Invalid Configuration", str(exc))
            return

        try:
            run_id, run_dir, images_dir = create_run_directory(config.out_root)
        except Exception as exc:
            QMessageBox.critical(self, "Directory Error", f"Failed to create output directory:\n{exc}")
            return

        cluster_settings = self.configuration_dialog.get_cluster_settings()
        osc_client = None
        if cluster_settings["enabled"]:
            try:
                osc_client = udp_client.SimpleUDPClient(cluster_settings["ip"], cluster_settings["port"])
                self.status_bar.showMessage(f"OSC client connected to {cluster_settings['ip']}:{cluster_settings['port']}")
            except Exception as exc:
                self.status_bar.showMessage(f"OSC connection failed: {exc}")
                osc_client = None

        self.current_run_dir = run_dir
        self.current_images_dir = images_dir
        self.current_t0 = time.perf_counter()
        self.stop_event = threading.Event()
        self.ensure_camera_preview_running(config)
        self.logging_active = True
        self.workers.clear()
        self.serial_threads.clear()

        for i, (port, baudrate) in enumerate(config.serial_ports or []):
            serial_name = f"serial{i+1}"
            self.series_data.setdefault(serial_name, {"time": deque(maxlen=10000), "cols": {}})

        self.serial1_count_label.setText("0")
        self.serial2_count_label.setText("0")
        self.image_count_label.setText("0")
        self.set_pose_labels(0.0, 0.0, 0.0)
        self.save_dir_label.setText(str(run_dir))
        self.set_recording_state(True)
        self.status_bar.showMessage(f"Recording to {run_dir}")
        self.recording_graph_start_ms = perf_ms(self.graph_t0)
        self.pending_recording_start_marker = True
        self.last_image_graph_time_ms = None

        self.write_run_info(run_id, run_dir, config)
        for i, (port, baudrate) in enumerate(config.serial_ports or []):
            self.start_serial_worker(
                f"serial{i+1}",
                port,
                baudrate,
                run_dir / f"serial{i+1}.csv",
                osc_client=osc_client,
                osc_path=cluster_settings.get("osc_path", "/plane/data"),
                send_fps=cluster_settings.get("send_fps", 50),
            )
        if self.preview_worker is not None:
            self.preview_worker.start_recording(
                str(run_dir / "image_log.csv"),
                str(images_dir),
                config.camera_backend or "",
                float(config.image_save_hz),
                int(config.jpeg_quality),
                self.current_t0,
            )

    def write_run_info(self, run_id: str, run_dir: Path, config: LoggingConfig) -> None:
        lines = [
            f"run_id: {run_id}",
            f"start_datetime: {pc_datetime()}",
        ]
        if config.serial_ports:
            for i, (port, baudrate) in enumerate(config.serial_ports):
                lines.append(f"serial{i+1}: {port} @ {baudrate} baud")
        else:
            lines.append(f"serial1: {config.serial1_port}")
            lines.append(f"serial2: {config.serial2_port}")
        lines.extend([
            f"camera_index: {config.camera_index}",
            f"camera_backend: {config.camera_backend or 'auto'}",
            f"image_save_hz: {config.image_save_hz}",
            f"jpeg_quality: {config.jpeg_quality}",
            f"camera_width: {config.camera_width}",
            f"camera_height: {config.camera_height}",
            f"camera_fps: {config.camera_fps}",
        ])
        (run_dir / "run_info.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def start_serial_worker(
        self,
        serial_name: str,
        port: str,
        baudrate: int,
        csv_path: Path,
        osc_client: udp_client.SimpleUDPClient | None = None,
        osc_path: str = "/plane/data",
        send_fps: int = 50,
    ) -> None:
        assert self.stop_event is not None
        worker = SerialWorker(
            serial_name,
            port,
            baudrate,
            csv_path,
            self.graph_t0,
            self.stop_event,
            csv_t0=self.current_t0,
            osc_client=osc_client,
            osc_path=osc_path,
            send_fps=send_fps,
        )
        thread = QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.sample_ready.connect(self.on_serial_sample)
        worker.line_count_changed.connect(self.on_line_count_changed)
        worker.status_changed.connect(self.status_bar.showMessage)
        worker.error.connect(self.on_worker_error)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(lambda t=thread: self._remove_serial_thread(t))
        thread.start()
        self.serial_threads.append(thread)
        self.workers.append(worker)

    def start_camera_worker(self, config: LoggingConfig, image_log_path: Path, images_dir: Path) -> None:
        assert self.stop_event is not None
        worker = CameraWorker(
            camera_index=config.camera_index,
            camera_backend=config.camera_backend,
            t0=self.current_t0,
            stop_event=self.stop_event,
            image_log_path=image_log_path,
            images_dir=images_dir,
            image_save_hz=config.image_save_hz,
            jpeg_quality=config.jpeg_quality,
            camera_width=config.camera_width,
            camera_height=config.camera_height,
            camera_fps=config.camera_fps,
        )
        thread = QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.image_saved.connect(self.on_image_saved)
        worker.status_changed.connect(self.status_bar.showMessage)
        worker.error.connect(self.on_worker_error)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_camera_thread)
        thread.start()
        self.camera_thread = thread
        self.workers.append(worker)

    def ensure_camera_preview_running(self, config: LoggingConfig | None = None) -> None:
        if self.preview_thread is not None and self.preview_worker is not None:
            return
        if config is None:
            try:
                config = self.collect_config()
            except Exception:
                return
        self.start_idle_preview(config)

    def start_idle_preview(self, config: LoggingConfig | None = None) -> None:
        if self.logging_active or self.is_closing:
            return

        if config is None:
            camera_text = self.configuration_dialog.camera_combo.currentText().strip()
            if not camera_text:
                return
            try:
                camera_index = int(camera_text)
            except ValueError:
                return
            camera_backend = self.camera_backend_map.get(camera_index)
        else:
            camera_index = config.camera_index
            camera_backend = config.camera_backend

        self.stop_idle_preview(clear_frame=False)
        self.preview_label.setText("Starting camera preview...")
        self.preview_started_once = True

        self.preview_stop_event = threading.Event()
        worker = CameraPreviewWorker(
            camera_index=camera_index,
            camera_backend=camera_backend,
            stop_event=self.preview_stop_event,
            camera_width=CAMERA_WIDTH,
            camera_height=CAMERA_HEIGHT,
            camera_fps=CAMERA_FPS,
        )
        thread = QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.preview_ready.connect(self.on_preview_ready)
        worker.image_saved.connect(self.on_image_saved)
        worker.image_saved_at.connect(self.on_image_saved_at)
        worker.status_changed.connect(self.status_bar.showMessage)
        worker.error.connect(self.on_preview_error)
        self.camera_start_requested.connect(worker.start_recording)
        self.camera_stop_requested.connect(worker.stop_recording)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_preview_thread)
        thread.start()
        self.preview_worker = worker
        self.preview_thread = thread

    def stop_idle_preview(self, clear_frame: bool = False) -> bool:
        if self.preview_stop_event is not None:
            self.preview_stop_event.set()
            self.preview_stop_event = None

        if stop_qthread(self.preview_thread, timeout_ms=5000):
            self.preview_thread = None
            self.preview_worker = None
        else:
            self.status_bar.showMessage("preview: still stopping, please try Start Logging again")
            return False

        if clear_frame:
            self.preview_label.setPixmap(QPixmap())
            self.preview_label.setText("Camera preview")
        return True

    def start_serial_preview(self) -> None:
        """Connect to serial ports without CSV logging after configuration is confirmed, starting real-time plot display."""
        if self.logging_active or self.is_closing:
            return

        if not self.stop_serial_preview():
            return

        serial_configs = self.configuration_dialog.get_serial_configs()
        if not serial_configs:
            return

        self.preview_serial_stop_event = threading.Event()
        if not self.series_data:
            self.graph_t0 = time.perf_counter()
        for i, (port, baudrate) in enumerate(serial_configs):
            serial_name = f"serial{i+1}"
            self.series_data.setdefault(serial_name, {"time": deque(maxlen=10000), "cols": {}})
            self._start_preview_serial_worker(serial_name, port, baudrate, self.graph_t0)

    def _start_preview_serial_worker(self, serial_name: str, port: str, baudrate: int, t0: float) -> None:
        assert self.preview_serial_stop_event is not None
        worker = SerialWorker(
            serial_name,
            port,
            baudrate,
            None,
            t0,
            self.preview_serial_stop_event,
        )
        thread = QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.sample_ready.connect(self.on_serial_sample)
        worker.line_count_changed.connect(self.on_line_count_changed)
        worker.status_changed.connect(self.status_bar.showMessage)
        worker.error.connect(self.on_preview_serial_error)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(lambda t=thread: self._remove_preview_serial_thread(t))
        thread.start()
        self.preview_serial_threads.append(thread)
        self.preview_serial_workers.append(worker)

    def stop_serial_preview(self) -> bool:
        if self.preview_serial_stop_event is not None:
            self.preview_serial_stop_event.set()
            self.preview_serial_stop_event = None

        remaining_threads = []
        for thread in list(self.preview_serial_threads):
            if not stop_qthread(thread, timeout_ms=3000):
                remaining_threads.append(thread)
        self.preview_serial_threads = remaining_threads
        if not remaining_threads:
            self.preview_serial_workers.clear()
        else:
            self.status_bar.showMessage("serial preview: still stopping, please try Start Logging again")
            return False
        return True

    def _remove_preview_serial_thread(self, thread: QThread) -> None:
        self.preview_serial_threads = [t for t in self.preview_serial_threads if t is not thread]

    @Slot(str)
    def on_preview_serial_error(self, message: str) -> None:
        self.status_bar.showMessage(message)

    def set_recording_state(self, active: bool) -> None:
        self.recording_label.setObjectName("recordingActive" if active else "recordingIdle")
        self.recording_label.setText("RECORDING" if active else "IDLE")
        self.recording_label.style().unpolish(self.recording_label)
        self.recording_label.style().polish(self.recording_label)
        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(active)

    @Slot(object)
    def on_serial_sample(self, sample: SerialSample) -> None:
        self.graph_queue.put(sample)
        if sample.serial_name == "serial1" and len(sample.cols) >= 3:
            roll = float(sample.cols[0]) if isinstance(sample.cols[0], (float, int)) else 0.0
            pitch = float(sample.cols[1]) if isinstance(sample.cols[1], (float, int)) else 0.0
            yaw = float(sample.cols[2]) if isinstance(sample.cols[2], (float, int)) else 0.0
            self.latest_pose = (roll, pitch, yaw)
            self.set_pose_labels(roll, pitch, yaw)

    @Slot(str, int)
    def on_line_count_changed(self, serial_name: str, count: int) -> None:
        if serial_name == "serial1":
            self.serial1_count_label.setText(str(count))
        else:
            self.serial2_count_label.setText(str(count))

    @Slot(QImage)
    def on_preview_ready(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.preview_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.preview_label.setText("")
        self.preview_label.setPixmap(scaled)

    @Slot(int)
    def on_image_saved(self, count: int) -> None:
        self.image_count_label.setText(str(count))

    @Slot(int, float)
    def on_image_saved_at(self, count: int, image_time_ms: float) -> None:
        self.image_count_label.setText(str(count))
        graph_time_ms = self.recording_graph_start_ms + image_time_ms
        self.last_image_graph_time_ms = graph_time_ms
        if self.pending_recording_start_marker:
            self._add_recording_marker("start", graph_time_ms)
            self.pending_recording_start_marker = False

    # High-contrast color palette supporting 15+ columns
    _CURVE_COLORS = [
        "#4ade80", "#38bdf8", "#facc15", "#f87171", "#a78bfa", "#06b6d4",
        "#fb923c", "#e879f9", "#34d399", "#f472b6", "#60a5fa", "#fbbf24",
        "#a3e635", "#818cf8", "#2dd4bf",
    ]

    @Slot()
    def drain_graph_queue(self) -> None:
        changed = False
        while True:
            try:
                sample = self.graph_queue.get_nowait()
            except Empty:
                break

            if sample.serial_name not in self.series_data:
                continue

            store = self.series_data[sample.serial_name]
            numeric_found = False
            for col_idx, val in enumerate(sample.cols):
                if not isinstance(val, (int, float)):
                    continue
                col_deque = store["cols"].setdefault(col_idx, deque(maxlen=10000))
                col_deque.append(float(val))
                numeric_found = True

            if numeric_found:
                store["time"].append(sample.pc_time_ms)
                changed = True

        if not changed:
            return

        for serial_name in self.selected_graphs:
            if serial_name not in self.plot_widgets or serial_name not in self.series_data:
                continue

            store = self.series_data[serial_name]
            t_vals = np.asarray(store["time"], dtype=float)
            if t_vals.size == 0:
                continue

            min_time = max(0.0, float(t_vals[-1]) - WINDOW_SECONDS * 1000.0)
            mask = t_vals >= min_time
            t_window = t_vals[mask]

            curves_dict = self.serial_curves.setdefault(serial_name, {})
            plot = self.plot_widgets[serial_name]

            for col_idx, val_deque in store["cols"].items():
                if col_idx not in curves_dict:
                    color = self._CURVE_COLORS[col_idx % len(self._CURVE_COLORS)]
                    curve = plot.plot(pen=pg.mkPen(color, width=1.5), name=f"col_{col_idx}")
                    curves_dict[col_idx] = curve

                y_vals = np.asarray(val_deque, dtype=float)
                if y_vals.size == len(t_vals):
                    curves_dict[col_idx].setData(t_window, y_vals[mask])

        latest_times = []
        for serial_name in self.selected_graphs:
            if serial_name in self.series_data and self.series_data[serial_name]["time"]:
                latest_times.append(self.series_data[serial_name]["time"][-1])
        if latest_times:
            right = max(latest_times)
            left = max(0.0, right - WINDOW_SECONDS * 1000.0)
            for serial_name in self.selected_graphs:
                if serial_name in self.plot_widgets:
                    plot = self.plot_widgets[serial_name]
                    plot.setXRange(left, right, padding=0.02)
                    self._position_recording_marker_labels(serial_name)

    def _position_recording_marker_labels(self, serial_name: str) -> None:
        plot = self.plot_widgets.get(serial_name)
        if plot is None:
            return
        try:
            y_min, y_max = plot.viewRange()[1]
        except Exception:
            return
        y_pos = y_max - (y_max - y_min) * 0.08
        items = self.recording_marker_items.get(serial_name, [])
        for item in items:
            if isinstance(item, pg.TextItem):
                x_pos = item.pos().x()
                item.setPos(x_pos, y_pos)

    def set_pose_labels(self, roll: float, pitch: float, yaw: float) -> None:
        self.roll_label.setText(f"{roll:.2f} deg")
        self.pitch_label.setText(f"{pitch:.2f} deg")
        self.yaw_label.setText(f"{yaw:.2f} deg")

    @Slot(str)
    def on_worker_error(self, message: str) -> None:
        self.status_bar.showMessage(message)
        QMessageBox.warning(self, "Worker Error", message)

    @Slot(str)
    def on_preview_error(self, message: str) -> None:
        self.status_bar.showMessage(message)
        self.preview_label.setPixmap(QPixmap())
        self.preview_label.setText(f"Camera preview unavailable\n{message}")

    @Slot(str)
    def on_camera_selection_changed(self, _value: str) -> None:
        if not self.logging_active:
            self.start_idle_preview()

    @Slot()
    def stop_logging(self) -> None:
        if not self.logging_active:
            return

        if self.stop_event is not None:
            self.stop_event.set()

        for thread in list(self.serial_threads):
            stop_qthread(thread, timeout_ms=3000)
        self.serial_threads.clear()

        if self.preview_worker is not None:
            self.preview_worker.stop_recording()
        if self.last_image_graph_time_ms is not None:
            if self.pending_recording_start_marker:
                self._add_recording_marker("start", self.last_image_graph_time_ms)
                self.pending_recording_start_marker = False
            self._add_recording_marker("stop", self.last_image_graph_time_ms)
        else:
            fallback_time_ms = perf_ms(self.graph_t0)
            if self.pending_recording_start_marker:
                self._add_recording_marker("start", fallback_time_ms)
                self.pending_recording_start_marker = False
            self._add_recording_marker("stop", fallback_time_ms)

        self.workers.clear()
        self.logging_active = False
        self.set_recording_state(False)
        self.status_bar.showMessage("Logging stopped")
        if not self.is_closing:
            self.start_serial_preview()

    @Slot()
    def browse_replay_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, "Select Data Folder", str(OUT_ROOT))
        if not folder:
            return

        folder_path = Path(folder)
        folder_name = folder_path.name
        self.replay_folder_label.setText(folder_name)
        self._load_replay_info(folder_path)

    def _load_replay_info(self, folder_path: Path) -> None:
        serial_files = list(folder_path.glob("serial*.csv"))
        if not serial_files:
            self.replay_count_label.setText("Data points: 0")
            self.replay_date_label.setText("Date range: No CSV files")
            self.play_button.setEnabled(False)
            return

        total_rows = 0
        min_datetime = None
        max_datetime = None

        for csv_file in serial_files:
            try:
                with csv_file.open("r", encoding="utf-8-sig") as f:
                    reader = csv.reader(f)
                    header = next(reader, None)
                    rows = list(reader)
                    total_rows += len(rows)

                    if rows:
                        try:
                            first_dt = rows[0][1] if len(rows[0]) > 1 else ""
                            last_dt = rows[-1][1] if len(rows[-1]) > 1 else ""
                            if first_dt and (min_datetime is None or first_dt < min_datetime):
                                min_datetime = first_dt
                            if last_dt and (max_datetime is None or last_dt > max_datetime):
                                max_datetime = last_dt
                        except Exception:
                            pass
            except Exception:
                pass

        self.replay_count_label.setText(f"Data points: {total_rows}")
        date_str = "-"
        if min_datetime and max_datetime:
            date_str = f"{min_datetime} to {max_datetime}"
        elif min_datetime:
            date_str = min_datetime
        self.replay_date_label.setText(f"Date range: {date_str}")

        if total_rows > 0:
            self.play_button.setEnabled(True)
            self.selected_replay_folder = folder_path
        else:
            self.play_button.setEnabled(False)
            self.selected_replay_folder = None

    @Slot()
    def start_replay(self) -> None:
        if not hasattr(self, "selected_replay_folder") or self.selected_replay_folder is None:
            QMessageBox.warning(self, "No Folder", "Please select a data folder first")
            return

        if self.replaying:
            return

        self.stop_logging()
        self.replaying = True
        self.play_button.setEnabled(False)
        self.stop_replay_button.setEnabled(True)
        self.replay_stop_event = threading.Event()
        self.workers.clear()
        self.graph_queue = Queue()
        self.series_data.clear()
        self.serial_curves.clear()
        self.recording_markers.clear()
        self._clear_recording_marker_items()
        self.series_data["serial1"] = {"time": deque(maxlen=10000), "cols": {}}

        folder_path = self.selected_replay_folder
        csv_files = sorted(folder_path.glob("serial*.csv"))

        cluster_settings = self.configuration_dialog.get_cluster_settings()
        osc_client = None
        if cluster_settings["enabled"]:
            try:
                osc_client = udp_client.SimpleUDPClient(cluster_settings["ip"], cluster_settings["port"])
            except Exception as exc:
                self.status_bar.showMessage(f"OSC connection failed: {exc}")
                osc_client = None

        self.status_bar.showMessage(f"Replaying from {folder_path.name}")
        self.replay_progress_label.setText("0 / 0")

        for csv_file in csv_files:
            self._start_replay_worker(csv_file, osc_client, cluster_settings)

    def _start_replay_worker(
        self,
        csv_path: Path,
        osc_client: udp_client.SimpleUDPClient | None,
        cluster_settings: dict,
    ) -> None:
        assert self.replay_stop_event is not None

        try:
            with csv_path.open("r", encoding="utf-8-sig") as f:
                reader = csv.reader(f)
                next(reader, None)
                row_count = sum(1 for _ in reader)
        except Exception:
            return

        if row_count == 0:
            return

        fps = 1.0 / (0.02) if row_count < 50 else min(30.0, row_count / 10.0)

        worker = ReplayWorker(
            csv_path=csv_path,
            original_fps=fps,
            osc_client=osc_client,
            osc_path=cluster_settings.get("osc_path", "/plane/data"),
            send_fps=cluster_settings.get("send_fps", 50),
            stop_event=self.replay_stop_event,
        )

        thread = QThread(self)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.sample_ready.connect(self.on_replay_sample)
        worker.progress_updated.connect(self.on_replay_progress)
        worker.status_changed.connect(self.status_bar.showMessage)
        worker.error.connect(self.on_worker_error)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.quit)
        thread.finished.connect(thread.deleteLater)
        thread.start()

        self.replay_thread = thread
        self.replay_worker = worker
        self.workers.append(worker)

    @Slot(object)
    def on_replay_sample(self, sample: SerialSample) -> None:
        replay_sample = SerialSample(
            serial_name="serial1",
            pc_time_ms=sample.pc_time_ms,
            pc_datetime=sample.pc_datetime,
            raw=sample.raw,
            cols=sample.cols,
        )
        self.graph_queue.put(replay_sample)
        if len(sample.cols) >= 3:
            roll = float(sample.cols[0]) if isinstance(sample.cols[0], (float, int)) else 0.0
            pitch = float(sample.cols[1]) if isinstance(sample.cols[1], (float, int)) else 0.0
            yaw = float(sample.cols[2]) if isinstance(sample.cols[2], (float, int)) else 0.0
            self.latest_pose = (roll, pitch, yaw)
            self.set_pose_labels(roll, pitch, yaw)

    @Slot(int, int)
    def on_replay_progress(self, current: int, total: int) -> None:
        self.replay_progress_label.setText(f"{current} / {total}")
        if current >= total:
            self.finish_replay()

    @Slot()
    def stop_replay(self) -> None:
        if not self.replaying:
            return

        if self.replay_stop_event is not None:
            self.replay_stop_event.set()

        stop_qthread(self.replay_thread, timeout_ms=3000)
        self.replay_thread = None
        self.replay_worker = None

        self.workers.clear()
        self.replaying = False
        self.play_button.setEnabled(True)
        self.stop_replay_button.setEnabled(False)
        self.status_bar.showMessage("Playback stopped")

    def finish_replay(self) -> None:
        self.replaying = False
        self.play_button.setEnabled(True)
        self.stop_replay_button.setEnabled(False)
        self.status_bar.showMessage("Playback finished")

    def closeEvent(self, event) -> None:
        self.is_closing = True
        self.stop_idle_preview(clear_frame=False)
        self.stop_serial_preview()
        self.stop_logging()
        super().closeEvent(event)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if not self.preview_started_once and not self.logging_active:
            self.preview_started_once = True
            QTimer.singleShot(0, self.start_idle_preview)

    def _remove_serial_thread(self, thread: QThread) -> None:
        self.serial_threads = [item for item in self.serial_threads if item is not thread]

    def _clear_camera_thread(self) -> None:
        self.camera_thread = None

    def _clear_preview_thread(self) -> None:
        self.preview_thread = None
        self.preview_worker = None


def build_app() -> QApplication:
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    font = app.font()
    if font.pointSize() <= 0:
        font.setPointSize(10)
        app.setFont(font)
    pg.setConfigOptions(antialias=True)
    return app


def camera_capture_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-index", type=int, required=True)
    parser.add_argument("--camera-backend")
    parser.add_argument("--image-log-path", type=Path, required=True)
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--image-save-hz", type=float, required=True)
    parser.add_argument("--jpeg-quality", type=int, required=True)
    parser.add_argument("--camera-width", type=int, required=True)
    parser.add_argument("--camera-height", type=int, required=True)
    parser.add_argument("--camera-fps", type=int, required=True)
    parser.add_argument("--t0", type=float, required=True)
    args = parser.parse_args(argv)

    cap = None
    try:
        args.images_dir.mkdir(parents=True, exist_ok=True)
        with args.image_log_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["image_index", "pc_time_ms", "pc_datetime", "filename"])
            last_flush = time.perf_counter()

            cap = open_camera_with_backend(args.camera_index, args.camera_backend, suppress_warnings=False)
            if cap is None:
                print(f"ERROR camera: failed to open camera index {args.camera_index}", flush=True)
                return 2

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
            cap.set(cv2.CAP_PROP_FPS, args.camera_fps)
            print(
                f"STATUS camera: connected to index {args.camera_index} via {args.camera_backend or 'auto'}",
                flush=True,
            )

            image_index = 0
            last_save = 0.0
            save_interval = 1.0 / max(args.image_save_hz, 0.01)

            while True:
                ok, frame = cap.read()
                if not ok:
                    print("ERROR camera: failed to read frame", flush=True)
                    return 3

                now = time.perf_counter()
                if now - last_save < save_interval:
                    time.sleep(0.002)
                    continue

                sample_time_ms = perf_ms(args.t0)
                sample_datetime = pc_datetime()
                filename = f"img_{image_index:06d}.jpg"
                filepath = args.images_dir / filename
                ok = cv2.imwrite(
                    str(filepath),
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(args.jpeg_quality)],
                )
                if not ok:
                    print(f"ERROR camera: failed to write {filepath}", flush=True)
                    return 4

                rel_path = f"images/{filename}"
                writer.writerow([image_index, f"{sample_time_ms:.3f}", sample_datetime, rel_path])
                if now - last_flush >= CSV_FLUSH_INTERVAL_SEC:
                    csv_file.flush()
                    last_flush = now
                image_index += 1
                last_save = now
                print(f"SAVED {image_index} {sample_time_ms:.3f}", flush=True)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"ERROR camera: {exc}", flush=True)
        return 1
    finally:
        if cap is not None:
            release_camera(cap)


def camera_service_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-index", type=int, required=True)
    parser.add_argument("--camera-backend")
    parser.add_argument("--camera-width", type=int, required=True)
    parser.add_argument("--camera-height", type=int, required=True)
    parser.add_argument("--camera-fps", type=int, required=True)
    args = parser.parse_args(argv)

    command_queue: Queue[dict] = Queue()

    def read_commands() -> None:
        for line in sys.stdin:
            try:
                command_queue.put(json.loads(line))
            except json.JSONDecodeError:
                print("ERROR camera: invalid command", flush=True)

    threading.Thread(target=read_commands, daemon=True).start()

    cap = None
    csv_file = None
    writer = None
    image_index = 0
    image_log_path = None
    images_dir = None
    image_save_hz = IMAGE_SAVE_HZ
    jpeg_quality = JPEG_QUALITY
    recording_t0 = 0.0
    last_flush = time.perf_counter()
    last_preview = 0.0
    last_save = 0.0
    recording = False

    try:
        cap = open_camera_with_backend(args.camera_index, args.camera_backend, suppress_warnings=True)
        if cap is None:
            print(f"ERROR camera: failed to open camera index {args.camera_index}", flush=True)
            return 2

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
        cap.set(cv2.CAP_PROP_FPS, args.camera_fps)
        print(f"STATUS preview: camera {args.camera_index} connected via {args.camera_backend or 'auto'}", flush=True)

        preview_interval = 1.0 / PREVIEW_FPS

        while True:
            while True:
                try:
                    command = command_queue.get_nowait()
                except Empty:
                    break

                if command.get("cmd") == "EXIT":
                    return 0
                if command.get("cmd") == "STOP":
                    recording = False
                    if csv_file is not None:
                        csv_file.flush()
                        csv_file.close()
                    csv_file = None
                    writer = None
                    print("STATUS camera: recording stopped", flush=True)
                if command.get("cmd") == "START":
                    if csv_file is not None:
                        csv_file.flush()
                        csv_file.close()
                    image_log_path = Path(command["image_log_path"])
                    images_dir = Path(command["images_dir"])
                    image_save_hz = float(command.get("image_save_hz", IMAGE_SAVE_HZ))
                    jpeg_quality = int(command.get("jpeg_quality", JPEG_QUALITY))
                    recording_t0 = float(command.get("t0", time.perf_counter()))
                    image_index = 0
                    images_dir.mkdir(parents=True, exist_ok=True)
                    csv_file = image_log_path.open("w", newline="", encoding="utf-8")
                    writer = csv.writer(csv_file)
                    writer.writerow(["image_index", "pc_time_ms", "pc_datetime", "filename"])
                    last_flush = time.perf_counter()
                    last_save = 0.0
                    recording = True
                    print("STATUS camera: recording started", flush=True)

            ok, frame = cap.read()
            if not ok:
                print("ERROR camera: failed to read frame", flush=True)
                return 3

            now = time.perf_counter()
            if recording and writer is not None and images_dir is not None:
                save_interval = 1.0 / max(image_save_hz, 0.01)
                if now - last_save >= save_interval:
                    sample_time_ms = perf_ms(recording_t0)
                    sample_datetime = pc_datetime()
                    filename = f"img_{image_index:06d}.jpg"
                    filepath = images_dir / filename
                    ok = cv2.imwrite(
                        str(filepath),
                        frame,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
                    )
                    if ok:
                        rel_path = f"images/{filename}"
                        writer.writerow([image_index, f"{sample_time_ms:.3f}", sample_datetime, rel_path])
                        if now - last_flush >= CSV_FLUSH_INTERVAL_SEC and csv_file is not None:
                            csv_file.flush()
                            last_flush = now
                        image_index += 1
                        print(f"SAVED {image_index} {sample_time_ms:.3f}", flush=True)
                    else:
                        print(f"ERROR camera: failed to write {filepath}", flush=True)
                    last_save = now

            active_preview_interval = 1.0 / RECORDING_PREVIEW_FPS if recording else preview_interval
            preview_due = now - last_preview >= active_preview_interval
            if preview_due:
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 70],
                )
                if ok:
                    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
                    print(f"PREVIEW {payload}", flush=True)
                last_preview = now

            time.sleep(0.001)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"ERROR camera: {exc}", flush=True)
        return 1
    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        if cap is not None:
            release_camera(cap)


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "--camera-capture-worker":
        return camera_capture_main(sys.argv[2:])
    if len(sys.argv) > 1 and sys.argv[1] == "--camera-service-worker":
        return camera_service_main(sys.argv[2:])

    app = build_app()
    window = DashboardWindow()
    window.show()
    window.raise_()
    window.activateWindow()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
