# Rock Detection Scripts

This folder contains utility scripts for model export, deployment, and inference workflows used in the Rock Detection module.

## Contents

- `camera_inference.py`: Live inference from webcam or video stream (detection or segmentation mode).
- `stream_inference.py`: Live segmentation-focused inference for webcam/IP camera feeds with reconnect handling.
- `file_inference.py`: Interactive inference for video files, image files, or image directories.
- `export_to_onnx.py`: Export YOLO `.pt` weights to ONNX format.
- `build_tensorrt_engine.sh`: Build TensorRT engine from ONNX on Jetson.
- `JETSON_DEPLOY.md`: Deployment notes for Jetson.

## Requirements

Install dependencies from the `Autonomous/Rock Detection` directory:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python torch
```

## Script Usage

### 1) camera_inference.py

Runs live inference from webcam index or stream URL.

```bash
python3 Scripts/camera_inference.py --mode segment --model Models/segment_L_v2.pt --source 0 --imgsz 640 --conf 0.40 --device 0
```

Key arguments:
- `--mode`: `detect` or `segment`
- `--model`: model path (`.pt`)
- `--source`: webcam index (`0`) or stream/video path
- `--cam-width`, `--cam-height`: requested camera resolution
- `--imgsz`, `--conf`, `--device`
- `--save`: save annotated output to `runs/live_detect/output.mp4`

### 2) stream_inference.py

Runs continuous segmentation inference for webcam or IP camera with auto-reconnect if stream drops.

```bash
python3 Scripts/stream_inference.py --model Models/segment_L_v2.pt --source "http://<iphone-stream-url>" --imgsz 960 --conf 0.40 --device 0
```

Key arguments:
- `--model`: model path (`.pt`, `.engine`, `.onnx`)
- `--source`: webcam index or stream URL
- `--frame-width`, `--frame-height`: resize before inference
- `--reconnect-delay`: reconnect wait in seconds
- `--save`: save to `runs/iphone_segment/output.mp4`

### 3) file_inference.py

Runs inference on local files or folders. Supports image browsing (`h` previous, `l` next, `q` quit) and video playback (`q` quit).

Single image:

```bash
python3 Scripts/file_inference.py --model Models/segment_M.pt --source /path/to/image.jpg --imgsz 960 --conf 0.40 --device 0
```

Image directory:

```bash
python3 Scripts/file_inference.py --model Models/segment_L_v1.pt --source /path/to/images --imgsz 960 --conf 0.40 --device 0
```

Video file:

```bash
python3 Scripts/file_inference.py --model Models/segment_L_v2.pt --source /path/to/video.mp4 --imgsz 960 --conf 0.40 --device 0
```

Useful options:
- `--half`: FP16 inference on CUDA
- `--oom-fallback-device`: fallback device on CUDA OOM (default `cpu`)
- `--oom-fallback-imgsz`: lower image size for OOM retry
- `--fps`: playback override for video sources

### 4) export_to_onnx.py

Example:

```bash
python3 Scripts/export_to_onnx.py --mode segment --weights Models/segment_L_v2.pt --imgsz 960 --opset 13 --simplify
```

### 5) build_tensorrt_engine.sh

Example (run on Jetson):

```bash
chmod +x Scripts/build_tensorrt_engine.sh
./Scripts/build_tensorrt_engine.sh --onnx segment_L_v2.onnx --engine segment_L_v2.engine --imgsz 960 --fp16
```

## Notes

- For segmentation visualization, confidence should generally be `0.40` or higher.
- Use `Models/` paths explicitly when running scripts from `Autonomous/Rock Detection`.
- Press `q` in OpenCV windows to quit.
