from __future__ import annotations

import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Run live YOLO inference (detection or segmentation) on webcam or video stream."
	)
	parser.add_argument(
		"--mode",
		type=str,
		choices=["detect", "segment"],
		default="detect",
		help="Inference mode: detect for bounding boxes, segment for masks",
	)
	parser.add_argument(
		"--model",
		type=str,
		default=None,
		help="Path to model (.pt). If omitted, uses detection.pt for detect mode and segment.pt for segment mode",
	)
	parser.add_argument(
		"--source",
		type=str,
		default="0",
		help="Camera index (e.g. 0) or stream/video path/URL",
	)
	parser.add_argument(
		"--cam-width",
		type=int,
		default=1920,
		help="Requested camera width (used only when source is a webcam index)",
	)
	parser.add_argument(
		"--cam-height",
		type=int,
		default=1080,
		help="Requested camera height (used only when source is a webcam index)",
	)
	parser.add_argument("--imgsz", type=int, default=640, help="Inference image size")
	parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
	parser.add_argument(
		"--device",
		type=str,
		default="0",
		help="Inference device: 0 for first CUDA GPU, cpu for CPU",
	)
	parser.add_argument(
		"--save",
		action="store_true",
		help="Save annotated output video to runs/live_detect/output.mp4",
	)
	return parser.parse_args()


def parse_source(source: str) -> int | str:
	return int(source) if source.isdigit() else source


def main() -> None:
	args = parse_args()

	default_model = "detection.pt" if args.mode == "detect" else "segment_L_v2.pt"
	model_path = Path(args.model or default_model)
	if not model_path.exists():
		raise FileNotFoundError(f"Model not found: {model_path.resolve()}")

	model = YOLO(str(model_path))
	source = parse_source(args.source)

	cap = cv2.VideoCapture(source)
	if not cap.isOpened():
		raise RuntimeError(f"Unable to open source: {args.source}")

	if isinstance(source, int):
		# Request a higher capture resolution for webcam sources.
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.cam_width)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cam_height)
		actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
		actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
		print(
			f"Requested camera resolution: {args.cam_width}x{args.cam_height} | "
			f"Actual: {actual_w}x{actual_h}"
		)

	writer = None
	out_path = None

	if args.save:
		out_dir = Path("runs/live_detect")
		out_dir.mkdir(parents=True, exist_ok=True)
		out_path = out_dir / "output.mp4"

	print("Live inference started. Press 'q' to quit.")
	print(f"Mode: {args.mode}")
	print(f"Model: {model_path.resolve()}")
	if args.mode == "segment" and args.conf < 0.40:
		print("Segment mode uses a minimum confidence of 0.40 for display.")

	try:
		while True:
			ok, frame = cap.read()
			if not ok:
				print("Stream ended or frame read failed.")
				break

			effective_conf = max(args.conf, 0.40) if args.mode == "segment" else args.conf

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

			window_title = "Stone Segmentation - YOLOv12L" if args.mode == "segment" else "Stone Detection - YOLOv12L"
			cv2.imshow(window_title, annotated)

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
