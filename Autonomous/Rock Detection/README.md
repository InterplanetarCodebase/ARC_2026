# Rock Detection

This module provides real-time rock detection and segmentation using YOLO models.
It includes:
- Live inference (`run.py`)
- Advanced inference scripts with resource monitoring (`Scripts/`)
- Trained model weights (`Models/`)
- Dataset references (`Datasets/`)
- Jetson deployment helpers (`Scripts/`)
- Training notebook (`train.ipynb`)

## Directory Structure

```text
Rock Detection/
├── run.py
├── train.ipynb
├── README.md
├── Datasets/
│   └── dataset.md
├── Models/
│   ├── detection.pt
│   ├── segment.pt
│   └── models.md
└── Scripts/
    ├── camera_inference.py
    ├── file_inference.py
    ├── stream_inference.py
    ├── resource_monitor.py
    ├── export_to_onnx.py
    ├── build_tensorrt_engine.sh
    └── JETSON_DEPLOY.md
```

## Requirements

Python 3.8+ is recommended.

Install dependencies:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python torch psutil nvidia-ml-py3
```

Or install from the project root using:

```bash
python3 -m pip install -r requirements.txt
```

## Quick Start

From this directory, run one of the following.

Detection mode (bounding boxes):

```bash
python3 run.py --mode detect --model Models/detection.pt --source 0 --imgsz 640 --conf 0.25 --device 0
```

Segmentation mode (masks):

```bash
python3 run.py --mode segment --model Models/segment.pt --source 0 --imgsz 640 --conf 0.40 --device 0
```

Notes:
- Press `q` to quit the live window.
- If `--model` is not provided, `run.py` defaults to `detection.pt` (detect) or `segment.pt` (segment) in the current directory. In this repo layout, pass model paths explicitly (for example `Models/segment.pt`).
- In segment mode, display confidence is clamped to a minimum of `0.40` by script logic.

## `run.py` Arguments

- `--mode`: `detect` or `segment`
- `--model`: model path (`.pt` or engine path if supported by Ultralytics runtime)
- `--source`: webcam index (`0`) or video/stream path
- `--cam-width`: requested camera width for webcam sources (default `1920`)
- `--cam-height`: requested camera height for webcam sources (default `1080`)
- `--imgsz`: inference image size (default `640`)
- `--conf`: confidence threshold (default `0.25`)
- `--device`: inference device (`0` for first CUDA device, `cpu` for CPU)
- `--save`: save annotated output to `runs/live_detect/output.mp4`

Example with saved output:

```bash
python3 run.py --mode detect --model Models/detection.pt --source 0 --save
```

## Advanced Inference Scripts

The `Scripts/` directory contains enhanced inference tools with resource monitoring and additional features:

### `camera_inference.py`

Live camera/stream inference with real-time resource monitoring and performance optimizations.

**Features:**
- Real-time CPU, RAM, and GPU usage overlay
- Half precision (FP16) inference support for reduced VRAM
- Configurable confidence thresholds for detection and segmentation
- Video output saving with custom paths
- Support for webcams, video files, and network streams

**Example usage:**

```bash
# Basic webcam inference
python3 Scripts/camera_inference.py --model Models/segment.pt --source 0 --imgsz 960

# With half precision and custom confidence
python3 Scripts/camera_inference.py --mode segment --model Models/segment.pt --source 0 --imgsz 960 --primary-conf 0.35 --device 0

# Save output video
python3 Scripts/camera_inference.py --model Models/detection.pt --source 0 --save --save-path output/detection.mp4
```

**Key arguments:**
- `--mode`: `detect` or `segment`
- `--model`: Model path (.pt/.engine/.onnx)
- `--source`: Camera index or stream URL
- `--imgsz`: Inference image size (default 640)
- `--primary-conf` / `--conf`: Primary confidence threshold (default 0.25)
- `--segment-min-conf`: Minimum confidence for segmentation mode (default 0.40)
- `--cam-width` / `--cam-height`: Camera resolution (default 1920x1080)
- `--save`: Save annotated output
- `--save-path`: Output video path (default `runs/live_detect/output.mp4`)
- `--monitor-interval`: Resource monitoring sample rate in seconds (default 0.4)

### `file_inference.py`

Interactive inference on images, videos, or image directories with keyboard navigation and OOM fallback.

**Features:**
- Interactive image viewer with keyboard controls (h: previous, l: next, q: quit)
- Video playback with real-time inference
- Directory batch processing
- Half precision (FP16) support
- Automatic CUDA OOM fallback to CPU
- Resource monitoring overlay
- Temporary file handling for processed outputs

**Example usage:**

```bash
# Run on single image
python3 Scripts/file_inference.py --model Models/segment.pt --source test_images/image.png --imgsz 960 --conf 0.40

# Run on image directory
python3 Scripts/file_inference.py --model Models/segment.pt --source test_images/ --imgsz 960 --half

# Run on video with half precision
python3 Scripts/file_inference.py --model Models/detection.pt --source video.mp4 --imgsz 960 --half --device 0
```

**Key arguments:**
- `--model`: Model path (.pt/.engine/.onnx)
- `--source`: Path to image, video, or directory
- `--imgsz`: Inference image size (default 960)
- `--conf`: Confidence threshold (default 0.40)
- `--half`: Enable FP16 inference for reduced VRAM
- `--device`: Inference device (default "0")
- `--oom-fallback-device`: Fallback device for CUDA OOM (default "cpu")
- `--oom-fallback-imgsz`: Reduced image size for OOM retry (default 640)
- `--fps`: Override video playback FPS (0 keeps source FPS)
- `--display-width` / `--display-height`: Display resolution for images (default 1280x720)

**Supported formats:**
- Images: `.jpg`, `.jpeg`, `.png`, `.bmp`, `.webp`, `.tif`, `.tiff`
- Videos: `.mp4`, `.mov`, `.avi`, `.mkv`, `.m4v`, `.wmv`, `.flv`

### `stream_inference.py`

Live segmentation from webcam or iPhone/network camera streams with automatic reconnection.

**Features:**
- Network stream support (RTSP, HTTP)
- Automatic reconnection on stream drops
- Frame resizing for consistent processing
- Resource monitoring overlay
- Video output saving

**Example usage:**

```bash
# Webcam inference
python3 Scripts/stream_inference.py --model Models/segment.pt --source 0 --imgsz 960

# iPhone camera stream (using app like IP Webcam)
python3 Scripts/stream_inference.py --model Models/segment.pt --source "rtsp://192.168.1.100:8080/h264_ulaw.sdp" --imgsz 960

# With custom frame size and save output
python3 Scripts/stream_inference.py --model Models/segment.pt --source 0 --frame-width 1280 --frame-height 720 --save
```

**Key arguments:**
- `--model`: Model path (.pt/.engine/.onnx)
- `--source`: Camera index or stream URL (rtsp/http)
- `--imgsz`: Inference image size (default 960)
- `--frame-width` / `--frame-height`: Resize incoming frames (default 1280x720)
- `--primary-conf` / `--conf`: Confidence threshold (default 0.40)
- `--segment-min-conf`: Minimum confidence floor (default 0.40)
- `--reconnect-delay`: Seconds to wait before reconnecting (default 2.0)
- `--save`: Save output video
- `--window`: Custom window title

### `resource_monitor.py`

Utility module providing CPU, RAM, and GPU monitoring with on-screen overlay display.

**Features:**
- Process-level CPU and RAM monitoring (via `psutil`)
- System GPU utilization and process GPU memory tracking (via `nvidia-ml-py3`/pynvml)
- Real-time overlay rendering on video frames
- Average usage statistics reporting
- OpenCV runtime configuration and log filtering

This module is automatically imported by the inference scripts and requires `psutil` and `nvidia-ml-py3` packages.

## Models

See `Models/models.md` for model notes.

Current status summary:
- `Models/detection.pt`: intended for rock detection
- `Models/segment.pt`: intended for rock instance segmentation
- Reported behavior: segmentation works at lower confidence; detection performs poorly for distant rocks

## Datasets

Dataset references are listed in `Datasets/dataset.md`:
- Rock Detection: https://www.kaggle.com/datasets/khajababa69/rock-detect
- Rock Segmentation: https://www.kaggle.com/datasets/khajababa69/segment-rock

## Jetson Deployment (ONNX and TensorRT)

Deployment helpers are available in `Scripts/`.

1. Export `.pt` to ONNX:

```bash
python3 Scripts/export_to_onnx.py --mode detect --weights Models/detection.pt --imgsz 960 --opset 13 --simplify
python3 Scripts/export_to_onnx.py --mode segment --weights Models/segment.pt --imgsz 960 --opset 13 --simplify
```

2. Build TensorRT engine on Jetson:

```bash
chmod +x Scripts/build_tensorrt_engine.sh
./Scripts/build_tensorrt_engine.sh --onnx detection.onnx --engine detection.engine --imgsz 960 --fp16
./Scripts/build_tensorrt_engine.sh --onnx segment.onnx --engine segment.engine --imgsz 960 --fp16
```

3. Run with exported engine (if supported by your Ultralytics setup):

```bash
python3 run.py --mode detect --model detection.engine --imgsz 960 --cam-width 1920 --cam-height 1080 --device 0
python3 run.py --mode segment --model segment.engine --imgsz 960 --cam-width 1920 --cam-height 1080 --device 0
```

For additional deployment notes, see `Scripts/JETSON_DEPLOY.md`.

## Training

Use `train.ipynb` for model training and experimentation.

## Troubleshooting

- Camera does not open:
	- Verify source index (`--source 0`, `--source 1`, and so on).
	- Confirm camera permissions and that no other app is using it.
- Low FPS:
	- Reduce `--imgsz` (for example `640` or `800`).
	- Use GPU with `--device 0`.
	- Enable `--half` flag for FP16 inference (camera_inference.py, file_inference.py).
- Missed far/small rocks:
	- Increase `--imgsz` (for example `960`) if compute allows.
	- Revisit training data balance and label quality.
- CUDA out of memory:
	- Enable `--half` flag for reduced VRAM usage.
	- Use `file_inference.py` which has automatic OOM fallback to CPU.
	- Reduce `--imgsz` to a lower value.
- Resource monitor not showing GPU stats:
	- Ensure `nvidia-ml-py3` is installed: `pip install nvidia-ml-py3`
	- Verify NVIDIA drivers are properly installed.
	- GPU monitoring only works on NVIDIA GPUs with CUDA support.

