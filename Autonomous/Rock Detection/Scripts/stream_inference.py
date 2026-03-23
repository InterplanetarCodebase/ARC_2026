from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Live segmentation inference from webcam or iPhone IP camera feed."
	)
	parser.add_argument(
		"--model",
		type=str,
		default="segment.pt",
		help="Path to segmentation model weights (.pt/.engine/.onnx)",
	)
	parser.add_argument(
		"--source",
		type=str,
		required=True,
		help="Camera source: webcam index (0) or iPhone stream URL (rtsp/http)",
	)
	parser.add_argument("--imgsz", type=int, default=960, help="Inference image size")
	parser.add_argument(
		"--frame-width",
		type=int,
		default=1280,
		help="Resize incoming frames to this width before inference/display",
	)
	parser.add_argument(
		"--frame-height",
		type=int,
		default=720,
		help="Resize incoming frames to this height before inference/display",
	)
	parser.add_argument(
		"--conf",
		type=float,
		default=0.40,
		help="Confidence threshold (minimum 0.40 is enforced for segmentation display)",
	)
	parser.add_argument(
		"--device",
		type=str,
		default="0",
		help="Inference device: 0 for first CUDA GPU, cpu for CPU",
	)
	parser.add_argument(
		"--window",
		type=str,
		default="Stone Segmentation - iPhone Live Feed",
		help="OpenCV display window title",
	)
	parser.add_argument(
		"--reconnect-delay",
		type=float,
		default=2.0,
		help="Seconds to wait before reconnecting when stream drops",
	)
	parser.add_argument(
		"--save",
		action="store_true",
		help="Save annotated stream to runs/iphone_segment/output.mp4",
	)
	return parser.parse_args()


def parse_source(source: str) -> int | str:
	return int(source) if source.isdigit() else source


def open_capture(source: int | str) -> cv2.VideoCapture:
	cap = cv2.VideoCapture(source)
	if not cap.isOpened():
		raise RuntimeError(f"Unable to open source: {source}")
	return cap


def main() -> None:
	args = parse_args()

	model_path = Path(args.model)
	if not model_path.exists():
		raise FileNotFoundError(f"Model not found: {model_path.resolve()}")

	model = YOLO(str(model_path))
	source = parse_source(args.source)
	effective_conf = max(args.conf, 0.40)

	writer = None
	out_path = None

	if args.save:
		out_dir = Path("runs/iphone_segment")
		out_dir.mkdir(parents=True, exist_ok=True)
		out_path = out_dir / "output.mp4"

	print("Live segmentation started. Press 'q' to quit.")
	print(f"Model: {model_path.resolve()}")
	print(f"Source: {args.source}")
	print(f"Frame resize: {args.frame_width}x{args.frame_height}")
	print(f"Confidence threshold in use: {effective_conf:.2f}")

	cap = open_capture(source)

	try:
		while True:
			ok, frame = cap.read()
			if not ok:
				print("Stream dropped. Reconnecting...")
				cap.release()
				time.sleep(args.reconnect_delay)
				cap = open_capture(source)
				continue

			if frame.shape[1] != args.frame_width or frame.shape[0] != args.frame_height:
				interpolation = cv2.INTER_AREA if frame.shape[1] > args.frame_width else cv2.INTER_LINEAR
				frame = cv2.resize(frame, (args.frame_width, args.frame_height), interpolation=interpolation)

			result = model.predict(
				source=frame,
				imgsz=args.imgsz,
				conf=effective_conf,
				device=args.device,
				verbose=False,
			)[0]

			annotated = result.plot()

			if args.save:
				if writer is None:
					fps = cap.get(cv2.CAP_PROP_FPS)
					fps = fps if fps and fps > 0 else 25.0
					h, w = annotated.shape[:2]
					fourcc = cv2.VideoWriter_fourcc(*"mp4v")
					writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
				writer.write(annotated)

			cv2.imshow(args.window, annotated)

			if cv2.waitKey(1) & 0xFF == ord("q"):
				break
	finally:
		cap.release()
		if writer is not None:
			writer.release()
			print(f"Saved output video: {out_path.resolve()}")
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
