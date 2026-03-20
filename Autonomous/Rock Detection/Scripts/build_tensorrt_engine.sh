#!/usr/bin/env bash
set -euo pipefail

# Build TensorRT engine from ONNX on Jetson using trtexec.
# Example:
#   ./build_tensorrt_engine.sh --onnx segment.onnx --engine segment.engine --imgsz 960 --fp16

ONNX_PATH=""
ENGINE_PATH=""
IMGSZ="960"
USE_FP16="0"
USE_INT8="0"
WORKSPACE_MB="2048"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --onnx)
      ONNX_PATH="$2"
      shift 2
      ;;
    --engine)
      ENGINE_PATH="$2"
      shift 2
      ;;
    --imgsz)
      IMGSZ="$2"
      shift 2
      ;;
    --fp16)
      USE_FP16="1"
      shift
      ;;
    --int8)
      USE_INT8="1"
      shift
      ;;
    --workspace)
      WORKSPACE_MB="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

if [[ -z "$ONNX_PATH" || -z "$ENGINE_PATH" ]]; then
  echo "Usage: $0 --onnx model.onnx --engine model.engine [--imgsz 960] [--fp16] [--int8] [--workspace 2048]"
  exit 1
fi

if ! command -v trtexec >/dev/null 2>&1; then
  echo "trtexec not found. Install TensorRT tools on Jetson first."
  exit 1
fi

if [[ ! -f "$ONNX_PATH" ]]; then
  echo "ONNX file not found: $ONNX_PATH"
  exit 1
fi

FLAGS=()
FLAGS+=("--onnx=$ONNX_PATH")
FLAGS+=("--saveEngine=$ENGINE_PATH")
FLAGS+=("--workspace=$WORKSPACE_MB")
FLAGS+=("--minShapes=images:1x3x${IMGSZ}x${IMGSZ}")
FLAGS+=("--optShapes=images:1x3x${IMGSZ}x${IMGSZ}")
FLAGS+=("--maxShapes=images:1x3x${IMGSZ}x${IMGSZ}")
FLAGS+=("--verbose")

if [[ "$USE_FP16" == "1" ]]; then
  FLAGS+=("--fp16")
fi

if [[ "$USE_INT8" == "1" ]]; then
  FLAGS+=("--int8")
fi

echo "Building TensorRT engine..."
trtexec "${FLAGS[@]}"

echo "Engine created: $ENGINE_PATH"
