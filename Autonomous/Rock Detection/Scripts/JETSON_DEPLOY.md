Jetson Orin Nano Deployment Guide (Ubuntu 22.04)

Goal
- Convert your YOLO weights to ONNX.
- Build TensorRT engine on Jetson.
- Run live inference using your existing run.py.

Important
- Build the TensorRT engine on the Jetson device itself for best compatibility.
- Keep export and runtime image size the same (for example 960).

1) Export PT to ONNX (on your training machine)
- Detection:
  python3 export_to_onnx.py --mode detect --weights detection.pt --imgsz 960 --opset 13 --simplify
- Segmentation:
  python3 export_to_onnx.py --mode segment --weights segment.pt --imgsz 960 --opset 13 --simplify

2) Copy files to Jetson
- Copy run.py, the ONNX file, and optionally build_tensorrt_engine.sh.

3) On Jetson, install runtime dependencies
- Make sure JetPack is installed with TensorRT.
- Install python packages for your app:
  python3 -m pip install --upgrade pip
  python3 -m pip install ultralytics opencv-python numpy

4) Build TensorRT engine on Jetson
- Make script executable once:
  chmod +x build_tensorrt_engine.sh

- Detection engine:
  ./build_tensorrt_engine.sh --onnx detection.onnx --engine detection.engine --imgsz 960 --fp16

- Segmentation engine:
  ./build_tensorrt_engine.sh --onnx segment.onnx --engine segment.engine --imgsz 960 --fp16

5) Run inference with TensorRT engine
- Detection:
  python3 run.py --mode detect --model detection.engine --imgsz 960 --cam-width 1920 --cam-height 1080

- Segmentation:
  python3 run.py --mode segment --model segment.engine --imgsz 960 --cam-width 1920 --cam-height 1080

Notes and tuning
- Your script already enforces minimum 0.40 confidence for segment mode display.
- If FPS is low, reduce imgsz to 800 or 640.
- If small/far rocks are missed, keep imgsz 960 and lower camera FPS if needed.
- Use --device 0 on Jetson for GPU execution.
