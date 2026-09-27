# Python Measurement & Telemetry Tools

[English](README_en.md) | [日本語](README.md)

PC 側で機体・地上側受信機からのテレメトリデータを受信・可視化・ログ記録し、機体へコマンドを送信するための Python ツール群です。
Windows 環境での実験計測向けに最適化されています。

## ファイル構成 (Files)

- `logger_gui.py`
  - 本体 GUI 測定ダッシュボード。`PySide6` を使った 1 画面構成で、2 系統シリアル受信、JPEG 保存付きカメラプレビュー、リアルタイムグラフ、姿勢数値表示をまとめて扱います。
  - 最優先は保存安定性です。CSV 保存と画像保存は GUI スレッドではなく worker thread で行い、描画負荷が高くてもデータ記録を落とさない設計です。
- `main_logger.py`
  - 軽量な CLI ロガー。ヘッドレス環境でも動作し、2 系統シリアル受信とカメラ画像撮影を行い、CSV / JPEG に記録します。
- `live_plot_from_csv.py`
  - 保存済み CSV の簡易ライブプロッタ。Matplotlib を使って記録中の最新 CSV データをリアルタイムにグラフ追従表示します。
- `viewer_serialsend.py`
  - リアルタイム姿勢・テレメトリ表示 & コマンド送信ビューア。子機シリアルポートと接続し、姿勢角・加速度・サーボ角のリアルタイム描画、キー入力による飛行モードや PID ゲイン・トリムのオンザフライ調整、OSC プロトコル送信に対応します。

## Install

Python 3.10 以上を使ってください。

```bash
pip install PySide6 pyqtgraph opencv-python pyserial numpy pandas matplotlib
```

補足:
- `pandas` は後処理や簡易読込用です。
- `viewer_serialsend.py` で OSC 送信機能を使用する場合は `python-osc` も追加してください (`pip install python-osc`)。

## COM Port Check

接続中の COM ポート確認:

```bash
python -m serial.tools.list_ports
```

pyqtgraph サンプル確認:

```bash
python -m pyqtgraph.examples
```

## 実行方法 (Run)

`host/python` ディレクトリから各スクリプトを実行できます:

### 1. GUI 測定ダッシュボード (`logger_gui.py`)

```bash
python logger_gui.py
```

起動後の基本手順:

1. `Serial1 Port` と `Serial2 Port` を選ぶ
2. `Camera` を選ぶ
3. `Baudrate`、`Image Save Hz`、`JPEG Quality` を必要に応じて調整する
   - `serial1` の `col_0`, `col_1`, `col_2` を `roll`, `pitch`, `yaw` として使用
4. `Start Logging` を押す
5. 実験終了後に `Stop Logging` を押す

姿勢表示の仕様:
- `serial1` の `col_0`, `col_1`, `col_2` を `roll`, `pitch`, `yaw` として使用します
- GUI 上では数値表示のみ行います

### 2. CLI ロガー (`main_logger.py`)

GUI を使わずにシリアルとカメラの記録を行う場合:

```bash
python main_logger.py --serial1 COM3 --serial2 COM4 --baud 115200 --camera 0
```

### 3. CSV ライブプロッタ (`live_plot_from_csv.py`)

保存中の最新 CSV ログをリアルタイムにグラフ描画する場合:

```bash
python live_plot_from_csv.py --column col_0 --window-ms 30000
```

### 4. コックピットビューア & コマンド送信 (`viewer_serialsend.py`)

子機 ESP32 を接続してテレメトリの HUD 描画およびキーボードによるリアルタイムコマンド送信を行う場合:

```bash
python viewer_serialsend.py --port COM3 --baud 115200 --save-csv telemetry.csv
```

- キーボードから `A`（Autoモード）、`M`（Manualモード）、`P`/`p`（ピッチゲイン）、`L`/`l`（左翼トリム）などを送信可能。
- `--osc-ip 127.0.0.1 --osc-port 9000` を指定することで、外部アプリへの OSC テレメトリ送信も可能です。

## Saved Files

各実行で以下を作成します。

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

時刻仕様:

- `pc_time_ms`: 実験開始からの経過ミリ秒
- 基準クロック: `time.perf_counter()`
- 小数第 3 位まで保存
- `pc_datetime`: ISO 形式ミリ秒付き

## Notes

- 初期 COM ポートは `COM3`, `COM4`、初期 baudrate は `115200` です
- シリアルは改行区切りで、カンマ、タブ、空白を含む行から数値を抽出して `col_0..col_19` に保存します
- ヘッダー解釈はしません
- グラフ表示は `serial1`, `serial2` の `col_0` を初期表示します
- 姿勢は `roll`, `pitch`, `yaw` の数値表示のみです
- グラフは直近 30 秒を表示します
- 動画保存はしません。指定 Hz で JPEG を保存します
- 停止時は worker に停止通知し、CSV flush、スレッド終了待機、カメラ release を行います
- 保存処理を最優先しているため、描画負荷が高い環境では GUI 更新レートが先に落ちることがあります
