# Rock Detection

This module provides real-time rock detection and segmentation using YOLO models.
It includes:
- Live inference (`run.py`)
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
		├── export_to_onnx.py
		├── build_tensorrt_engine.sh
		└── JETSON_DEPLOY.md
```

## Requirements

Python 3.8+ is recommended.

Install dependencies:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python
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
- Missed far/small rocks:
	- Increase `--imgsz` (for example `960`) if compute allows.
	- Revisit training data balance and label quality.
