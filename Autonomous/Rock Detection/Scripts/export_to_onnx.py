from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export YOLO weights (.pt) to ONNX for Jetson deployment."
    )
    parser.add_argument(
        "--mode",
        choices=["detect", "segment"],
        default="detect",
        help="Model type used only for default weight filename.",
    )
    parser.add_argument(
        "--weights",
        type=str,
        default=None,
        help="Path to .pt weights. Defaults: detection.pt (detect) or segment.pt (segment)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=960,
        help="Export image size (square). Keep same as runtime for best performance.",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=13,
        help="ONNX opset version.",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Enable dynamic input shape in ONNX. Usually slower than static on Jetson.",
    )
    parser.add_argument(
        "--simplify",
        action="store_true",
        help="Simplify ONNX graph after export.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="0",
        help="Export device, for example 0 or cpu.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    default_weights = "detection.pt" if args.mode == "detect" else "segment.pt"
    weights_path = Path(args.weights or default_weights)
    if not weights_path.exists():
        raise FileNotFoundError(f"Weights not found: {weights_path.resolve()}")

    model = YOLO(str(weights_path))
    onnx_path = Path(
        model.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.opset,
            dynamic=args.dynamic,
            simplify=args.simplify,
            device=args.device,
        )
    )

    print(f"ONNX export complete: {onnx_path.resolve()}")


if __name__ == "__main__":
    main()
