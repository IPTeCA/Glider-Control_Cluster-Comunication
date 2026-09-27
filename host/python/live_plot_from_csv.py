#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except Exception as exc:
    print("matplotlib is required. Install with: pip install matplotlib", file=sys.stderr)
    raise

try:
    import pandas as pd
except Exception as exc:
    print("pandas is required. Install with: pip install pandas", file=sys.stderr)
    raise


PLOT_COLUMN = "col_0"
WINDOW_MS = 30_000
UPDATE_INTERVAL_MS = 500
OUT_ROOT = Path("data")


def find_latest_run(root: Path) -> Path:
    candidates = [path for path in root.iterdir() if path.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"No run directories found under {root}")
    return max(candidates, key=lambda path: path.name)


def load_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["pc_time_ms", PLOT_COLUMN])

    df = pd.read_csv(path)
    if "pc_time_ms" not in df.columns or PLOT_COLUMN not in df.columns:
        raise ValueError(f"{path} is missing required columns.")

    df["pc_time_ms"] = pd.to_numeric(df["pc_time_ms"], errors="coerce")
    df[PLOT_COLUMN] = pd.to_numeric(df[PLOT_COLUMN], errors="coerce")
    df = df.dropna(subset=["pc_time_ms", PLOT_COLUMN])
    return df


def load_image_log(path: Path) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=["image_index", "pc_time_ms", "pc_datetime", "filename"])

    df = pd.read_csv(path)
    if "pc_time_ms" in df.columns:
        df["pc_time_ms"] = pd.to_numeric(df["pc_time_ms"], errors="coerce")
    return df


def nearest_image_text(image_log: pd.DataFrame, target_time_ms: float) -> str:
    if image_log.empty or "pc_time_ms" not in image_log.columns:
        return "Nearest image: none"

    nearest = image_log.iloc[(image_log["pc_time_ms"] - target_time_ms).abs().argmin()]
    filename = nearest.get("filename", "")
    image_index = nearest.get("image_index", "")
    image_time = nearest.get("pc_time_ms", "")
    return f"Nearest image: #{image_index} @ {image_time:.1f} ms ({filename})"


def main() -> int:
    parser = argparse.ArgumentParser(description="Live plot latest serial CSV logs.")
    parser.add_argument("--run-dir", type=Path, default=None, help="Run directory under data/. Defaults to latest.")
    parser.add_argument("--column", type=str, default=PLOT_COLUMN, help="Column to plot. Default: col_0")
    parser.add_argument("--window-ms", type=int, default=WINDOW_MS, help="Visible time window in ms.")
    parser.add_argument("--interval-ms", type=int, default=UPDATE_INTERVAL_MS, help="Refresh interval in ms.")
    args = parser.parse_args()

    try:
        run_dir = args.run_dir if args.run_dir else find_latest_run(OUT_ROOT)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    serial1_path = run_dir / "serial1.csv"
    serial2_path = run_dir / "serial2.csv"
    image_log_path = run_dir / "image_log.csv"

    print(f"[INFO] Plotting run: {run_dir}")

    fig, ax = plt.subplots(figsize=(12, 6))
    line1, = ax.plot([], [], label="serial1", linewidth=2.0)
    line2, = ax.plot([], [], label="serial2", linewidth=2.0)
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.set_xlabel("pc_time_ms")
    ax.set_ylabel(args.column)
    ax.set_title(f"Latest run live plot: {run_dir.name}")
    ax.legend(loc="upper left")

    image_text = ax.text(
        0.01,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "#999999"},
    )

    def update(_frame_index: int) -> None:
        df1 = load_csv(serial1_path)
        df2 = load_csv(serial2_path)
        image_log = load_image_log(image_log_path)

        latest_time = None
        if not df1.empty:
            latest_time = float(df1["pc_time_ms"].iloc[-1])
        if not df2.empty:
            latest_time = max(latest_time or float("-inf"), float(df2["pc_time_ms"].iloc[-1]))

        if latest_time is None:
            ax.set_title(f"Latest run live plot: {run_dir.name} (waiting for data)")
            line1.set_data([], [])
            line2.set_data([], [])
            image_text.set_text("Nearest image: none")
            return

        min_time = max(0.0, latest_time - args.window_ms)

        window1 = df1[df1["pc_time_ms"] >= min_time]
        window2 = df2[df2["pc_time_ms"] >= min_time]

        if args.column not in df1.columns or args.column not in df2.columns:
            raise ValueError(f"Column {args.column!r} is not present in both serial CSV files.")

        line1.set_data(window1["pc_time_ms"], window1[args.column])
        line2.set_data(window2["pc_time_ms"], window2[args.column])

        ax.set_xlim(min_time, latest_time + 100.0)

        combined = pd.concat([window1[args.column], window2[args.column]], ignore_index=True).dropna()
        if combined.empty:
            ax.set_ylim(-1.0, 1.0)
        else:
            ymin = float(combined.min())
            ymax = float(combined.max())
            if ymin == ymax:
                ymin -= 1.0
                ymax += 1.0
            pad = (ymax - ymin) * 0.1
            ax.set_ylim(ymin - pad, ymax + pad)

        image_text.set_text(nearest_image_text(image_log, latest_time))

    FuncAnimation(fig, update, interval=args.interval_ms, cache_frame_data=False)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
