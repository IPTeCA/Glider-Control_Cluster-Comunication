#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import re
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from typing import Iterable

cv2 = None
cv2_import_error = None
try:
    import cv2 as _cv2
    cv2 = _cv2
except Exception as exc:
    cv2_import_error = exc

serial = None
list_ports = None
serial_import_error = None
try:
    import serial as _serial
    from serial.tools import list_ports as _list_ports

    serial = _serial
    list_ports = _list_ports
except Exception as exc:
    serial_import_error = exc


BAUDRATE = 115200

SERIAL_PORTS = {
    "serial1": "COM3",
    "serial2": "COM4",
}

MAX_SERIAL_COLUMNS = 20

CAMERA_SCAN_MAX = 5
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15

IMAGE_SAVE_HZ = 1
DISPLAY_HZ = 5
JPEG_QUALITY = 80

CSV_FLUSH_INTERVAL_SEC = 0.5

OUT_ROOT = Path("data")

WINDOW_NAME = "main_logger preview"
NUMERIC_PATTERN = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def perf_ms(t0: float) -> float:
    return (time.perf_counter() - t0) * 1000.0


def iso_now() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def ensure_flush(file_obj, last_flush: float) -> float:
    now = time.perf_counter()
    if now - last_flush >= CSV_FLUSH_INTERVAL_SEC:
        file_obj.flush()
        return now
    return last_flush


def split_numeric_tokens(raw: str) -> list[str]:
    return NUMERIC_PATTERN.findall(raw)


@dataclass
class SerialMessage:
    serial_name: str
    pc_time_ms: float
    pc_datetime: str
    raw: str
    values: list[object]


class SerialReaderThread(threading.Thread):
    def __init__(
        self,
        serial_name: str,
        port: str,
        baudrate: int,
        t0: float,
        out_queue: Queue[SerialMessage],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.serial_name = serial_name
        self.port = port
        self.baudrate = baudrate
        self.t0 = t0
        self.out_queue = out_queue
        self.stop_event = stop_event

    def run(self) -> None:
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.2) as ser:
                print(f"[INFO] {self.serial_name}: connected to {self.port} @ {self.baudrate}")
                while not self.stop_event.is_set():
                    try:
                        raw_bytes = ser.readline()
                    except serial.SerialException as exc:
                        print(f"[ERROR] {self.serial_name}: serial read failed: {exc}")
                        return

                    if not raw_bytes:
                        continue

                    raw = raw_bytes.decode("utf-8", errors="replace").strip()
                    if not raw:
                        continue

                    tokens = split_numeric_tokens(raw)[:MAX_SERIAL_COLUMNS]
                    values: list[object] = []
                    for token in tokens:
                        try:
                            values.append(float(token))
                        except ValueError:
                            values.append(token)

                    self.out_queue.put(
                        SerialMessage(
                            serial_name=self.serial_name,
                            pc_time_ms=round(perf_ms(self.t0), 3),
                            pc_datetime=iso_now(),
                            raw=raw,
                            values=values,
                        )
                    )
        except serial.SerialException as exc:
            print(f"[ERROR] {self.serial_name}: could not open {self.port}: {exc}")


def list_available_ports() -> list[str]:
    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


def build_serial_header() -> list[str]:
    return ["pc_time_ms", "pc_datetime", "raw"] + [f"col_{index}" for index in range(MAX_SERIAL_COLUMNS)]


def write_serial_row(writer: csv.writer, message: SerialMessage) -> None:
    padded = list(message.values[:MAX_SERIAL_COLUMNS])
    padded.extend([""] * (MAX_SERIAL_COLUMNS - len(padded)))
    writer.writerow([f"{message.pc_time_ms:.3f}", message.pc_datetime, message.raw, *padded])


def scan_camera_indices(max_index: int) -> list[int]:
    found: list[int] = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(index)
        cap.release()
    return found


def choose_camera_index() -> int:
    available = scan_camera_indices(CAMERA_SCAN_MAX)
    if not available:
        raise RuntimeError("No camera was found in indices 0-4.")

    print("Available cameras:")
    for index in available:
        print(f"  [{index}] Camera {index}")

    default_index = available[0]
    answer = input(f"\nUse camera index {default_index}? Enter index or blank: ").strip()
    if answer == "":
        return default_index

    camera_index = int(answer)
    if camera_index not in available:
        raise ValueError(f"Camera index {camera_index} is not available.")
    return camera_index


def open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera index {index}.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    return cap


def create_run_dirs() -> tuple[str, Path, Path]:
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = OUT_ROOT / run_id
    images_dir = run_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    return run_id, run_dir, images_dir


def write_run_info(run_dir: Path, run_id: str, camera_index: int, args: argparse.Namespace) -> None:
    lines = [
        f"run_id: {run_id}",
        f"start_datetime: {iso_now()}",
        f"serial1: {args.serial1_port}",
        f"serial2: {args.serial2_port}",
        f"baudrate: {args.baudrate}",
        f"camera_index: {camera_index}",
        f"image_save_hz: {args.image_save_hz}",
        f"display_hz: {args.display_hz}",
        f"jpeg_quality: {args.jpeg_quality}",
        f"camera_width: {args.camera_width}",
        f"camera_height: {args.camera_height}",
        f"camera_fps: {args.camera_fps}",
    ]
    (run_dir / "run_info.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def open_csv_writer(path: Path, header: Iterable[str]) -> tuple[object, csv.writer]:
    file_obj = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(file_obj)
    writer.writerow(list(header))
    return file_obj, writer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Log two serial streams and a camera preview into timestamped CSV/JPEG files."
    )
    parser.add_argument("--serial1-port", default=SERIAL_PORTS["serial1"], help="COM port for serial1.")
    parser.add_argument("--serial2-port", default=SERIAL_PORTS["serial2"], help="COM port for serial2.")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help="Serial baudrate.")
    parser.add_argument("--camera-scan-max", type=int, default=CAMERA_SCAN_MAX, help="Exclusive upper bound for camera scan.")
    parser.add_argument("--camera-width", type=int, default=CAMERA_WIDTH, help="Requested camera width.")
    parser.add_argument("--camera-height", type=int, default=CAMERA_HEIGHT, help="Requested camera height.")
    parser.add_argument("--camera-fps", type=int, default=CAMERA_FPS, help="Requested camera FPS.")
    parser.add_argument("--image-save-hz", type=float, default=IMAGE_SAVE_HZ, help="JPEG save frequency.")
    parser.add_argument("--display-hz", type=float, default=DISPLAY_HZ, help="Preview refresh frequency.")
    parser.add_argument("--jpeg-quality", type=int, default=JPEG_QUALITY, help="JPEG quality for saved images.")
    parser.add_argument("--out-root", type=Path, default=OUT_ROOT, help="Root directory for output runs.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    global CAMERA_SCAN_MAX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, IMAGE_SAVE_HZ, DISPLAY_HZ, JPEG_QUALITY, OUT_ROOT
    CAMERA_SCAN_MAX = args.camera_scan_max
    CAMERA_WIDTH = args.camera_width
    CAMERA_HEIGHT = args.camera_height
    CAMERA_FPS = args.camera_fps
    IMAGE_SAVE_HZ = args.image_save_hz
    DISPLAY_HZ = args.display_hz
    JPEG_QUALITY = args.jpeg_quality
    OUT_ROOT = args.out_root

    if serial is None:
        print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
        if serial_import_error:
            print(f"[DETAIL] {serial_import_error}", file=sys.stderr)
        return 1

    if cv2 is None:
        print("opencv-python is required. Install with: pip install opencv-python", file=sys.stderr)
        if cv2_import_error:
            print(f"[DETAIL] {cv2_import_error}", file=sys.stderr)
        return 1

    print("[INFO] Listing serial ports...")
    ports = list_available_ports()
    if ports:
        for port in ports:
            print(f"  {port}")
    else:
        print("  (no serial ports found)")

    try:
        camera_index = choose_camera_index()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    SERIAL_PORTS["serial1"] = args.serial1_port
    SERIAL_PORTS["serial2"] = args.serial2_port

    run_id, run_dir, images_dir = create_run_dirs()
    write_run_info(run_dir, run_id, camera_index, args)

    serial_header = build_serial_header()
    serial1_file, serial1_writer = open_csv_writer(run_dir / "serial1.csv", serial_header)
    serial2_file, serial2_writer = open_csv_writer(run_dir / "serial2.csv", serial_header)
    image_log_file, image_log_writer = open_csv_writer(
        run_dir / "image_log.csv",
        ["image_index", "pc_time_ms", "pc_datetime", "filename"],
    )

    stop_event = threading.Event()
    queue: Queue[SerialMessage] = Queue()
    t0 = time.perf_counter()

    threads = [
        SerialReaderThread("serial1", SERIAL_PORTS["serial1"], args.baudrate, t0, queue, stop_event),
        SerialReaderThread("serial2", SERIAL_PORTS["serial2"], args.baudrate, t0, queue, stop_event),
    ]
    for thread in threads:
        thread.start()

    try:
        cap = open_camera(camera_index)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        stop_event.set()
        for thread in threads:
            thread.join(timeout=1.0)
        serial1_file.close()
        serial2_file.close()
        image_log_file.close()
        return 1

    print(f"[INFO] Logging to {run_dir}")
    print("[INFO] Press q in the preview window to stop.")

    last_serial1_flush = time.perf_counter()
    last_serial2_flush = time.perf_counter()
    last_image_flush = time.perf_counter()
    last_preview = 0.0
    last_image_save = 0.0
    image_index = 0
    preview_interval = 1.0 / DISPLAY_HZ
    image_interval = 1.0 / IMAGE_SAVE_HZ

    try:
        while not stop_event.is_set():
            while True:
                try:
                    message = queue.get_nowait()
                except Empty:
                    break

                if message.serial_name == "serial1":
                    write_serial_row(serial1_writer, message)
                    last_serial1_flush = ensure_flush(serial1_file, last_serial1_flush)
                else:
                    write_serial_row(serial2_writer, message)
                    last_serial2_flush = ensure_flush(serial2_file, last_serial2_flush)

            ok, frame = cap.read()
            if not ok:
                print("[ERROR] Failed to read a frame from the camera.")
                break

            now = time.perf_counter()
            if now - last_preview >= preview_interval:
                cv2.imshow(WINDOW_NAME, frame)
                last_preview = now

            if now - last_image_save >= image_interval:
                current_pc_time_ms = round(perf_ms(t0), 3)
                current_pc_datetime = iso_now()
                rel_filename = f"images/img_{image_index:06d}.jpg"
                image_path = run_dir / rel_filename
                cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                image_log_writer.writerow([image_index, f"{current_pc_time_ms:.3f}", current_pc_datetime, rel_filename])
                last_image_flush = ensure_flush(image_log_file, last_image_flush)
                image_index += 1
                last_image_save = now

            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_event.set()

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
        stop_event.set()
    finally:
        stop_event.set()
        for thread in threads:
            thread.join(timeout=1.0)

        serial1_file.flush()
        serial2_file.flush()
        image_log_file.flush()
        serial1_file.close()
        serial2_file.close()
        image_log_file.close()
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
