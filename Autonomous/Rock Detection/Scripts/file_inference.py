from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO


os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}
VIDEO_EXTS = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".wmv", ".flv"}
DISPLAY_SIZE = (1280, 720)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Run interactive YOLO inference on a video file or a directory of images."
	)
	parser.add_argument(
		"--model",
		type=str,
		default="segment.pt",
		help="Path to model (.pt/.engine/.onnx)",
	)
	parser.add_argument(
		"--source",
		type=str,
		required=True,
		help="Path to a video file, image file, or directory of images",
	)
	parser.add_argument("--imgsz", type=int, default=960, help="Inference image size")
	parser.add_argument("--conf", type=float, default=0.40, help="Confidence threshold")
	parser.add_argument(
		"--device",
		type=str,
		default="0",
		help="Inference device: 0 for first CUDA GPU, cpu for CPU",
	)
	parser.add_argument(
		"--half",
		action="store_true",
		help="Use FP16 inference on CUDA to reduce VRAM usage",
	)
	parser.add_argument(
		"--oom-fallback-device",
		type=str,
		default="cpu",
		help="Device used if CUDA runs out of memory",
	)
	parser.add_argument(
		"--oom-fallback-imgsz",
		type=int,
		default=640,
		help="Image size used when retrying after CUDA OOM",
	)
	parser.add_argument(
		"--fps",
		type=float,
		default=0.0,
		help="Override playback FPS for videos (0 keeps source FPS)",
	)
	return parser.parse_args()


def is_video(path: Path) -> bool:
	return path.suffix.lower() in VIDEO_EXTS


def is_image(path: Path) -> bool:
	return path.suffix.lower() in IMAGE_EXTS


def predict_with_fallback(model: YOLO, frame, args: argparse.Namespace):
	try:
		return model.predict(
			source=frame,
			imgsz=args.imgsz,
			conf=args.conf,
			device=args.device,
			half=args.half,
			verbose=False,
		)[0]
	except torch.OutOfMemoryError:
		if str(args.device).lower() == "cpu":
			raise
		print(
			"CUDA out of memory. Retrying with "
			f"device={args.oom_fallback_device}, imgsz={args.oom_fallback_imgsz}."
		)
		if torch.cuda.is_available():
			torch.cuda.empty_cache()
		return model.predict(
			source=frame,
			imgsz=args.oom_fallback_imgsz,
			conf=args.conf,
			device=args.oom_fallback_device,
			half=False,
			verbose=False,
		)[0]


def run_on_video(model: YOLO, source: Path, temp_dir: Path, args: argparse.Namespace) -> None:
	cap = cv2.VideoCapture(str(source))
	if not cap.isOpened():
		raise RuntimeError(f"Unable to open video: {source}")

	temp_dir.mkdir(parents=True, exist_ok=True)
	out_path = temp_dir / f"{source.stem}_annotated.mp4"
	writer = None

	frame_count = 0
	window_name = "Interactive Video Inference (q: quit)"
	src_fps = cap.get(cv2.CAP_PROP_FPS)
	fps = args.fps if args.fps > 0 else (src_fps if src_fps and src_fps > 0 else 25.0)
	wait_ms = max(1, int(1000 / fps))
	try:
		while True:
			ok, frame = cap.read()
			if not ok:
				break

			result = predict_with_fallback(model, frame, args)
			annotated = result.plot()

			if writer is None:
				h, w = annotated.shape[:2]
				fourcc = cv2.VideoWriter_fourcc(*"mp4v")
				writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))

			writer.write(annotated)
			cv2.imshow(window_name, annotated)
			if cv2.waitKey(wait_ms) & 0xFF == ord("q"):
				break
			frame_count += 1
	finally:
		cap.release()
		if writer is not None:
			writer.release()
		cv2.destroyAllWindows()

	if frame_count == 0:
		raise RuntimeError(f"No frames were read from video: {source}")


def build_annotated_images(
	model: YOLO, image_paths: list[Path], temp_dir: Path, args: argparse.Namespace
) -> list[Path]:
	temp_dir.mkdir(parents=True, exist_ok=True)
	annotated_paths: list[Path] = []

	for idx, image_path in enumerate(image_paths):
		frame = cv2.imread(str(image_path))
		if frame is None:
			print(f"Skipping unreadable image: {image_path}")
			continue

		result = predict_with_fallback(model, frame, args)
		annotated = result.plot()

		out_path = temp_dir / f"{idx:05d}_{image_path.name}"
		ok = cv2.imwrite(str(out_path), annotated)
		if not ok:
			print(f"Failed to write image: {out_path}")
			continue
		annotated_paths.append(out_path)

	return annotated_paths


def interactive_image_view(annotated_paths: list[Path]) -> None:
	if not annotated_paths:
		raise RuntimeError("No annotated images available to display.")

	index = 0
	window_name = "Interactive Image Inference (h: prev, l: next, q: quit)"

	while True:
		img = cv2.imread(str(annotated_paths[index]))
		if img is None:
			raise RuntimeError(f"Failed to open annotated image: {annotated_paths[index]}")
		img = cv2.resize(img, DISPLAY_SIZE, interpolation=cv2.INTER_AREA)

		title = f"{window_name} [{index + 1}/{len(annotated_paths)}]"
		cv2.imshow(title, img)
		key = cv2.waitKey(0) & 0xFF
		cv2.destroyWindow(title)

		if key == ord("q"):
			break
		if key == ord("l"):
			index = min(index + 1, len(annotated_paths) - 1)
			continue
		if key == ord("h"):
			index = max(index - 1, 0)
			continue

	cv2.destroyAllWindows()


def main() -> None:
	args = parse_args()

	model_path = Path(args.model)
	if not model_path.exists():
		raise FileNotFoundError(f"Model not found: {model_path.resolve()}")

	source_path = Path(args.source)
	if not source_path.exists():
		raise FileNotFoundError(f"Source not found: {source_path.resolve()}")
	
	model = YOLO(str(model_path))
	print(
		f"Inference settings -> device={args.device}, imgsz={args.imgsz}, half={args.half}"
	)

	with tempfile.TemporaryDirectory(prefix="file_inference_") as temp_root:
		temp_dir = Path(temp_root)
		print(f"Using temporary folder: {temp_dir}")

		if source_path.is_dir():
			image_paths = sorted(
				p for p in source_path.iterdir() if p.is_file() and is_image(p)
			)
			if not image_paths:
				raise RuntimeError(f"No supported images found in directory: {source_path.resolve()}")

			print(f"Running inference on {len(image_paths)} images...")
			annotated_paths = build_annotated_images(model, image_paths, temp_dir, args)
			print("Ready. Use 'h' for previous, 'l' for next, 'q' to quit.")
			interactive_image_view(annotated_paths)
			return

		if source_path.is_file() and is_image(source_path):
			print("Running inference on single image...")
			annotated_paths = build_annotated_images(model, [source_path], temp_dir, args)
			print("Ready. Use 'h' for previous, 'l' for next, 'q' to quit.")
			interactive_image_view(annotated_paths)
			return

		if source_path.is_file() and is_video(source_path):
			print("Running interactive inference on video... (press 'q' to quit)")
			run_on_video(model, source_path, temp_dir, args)
			return

		raise RuntimeError(
			"Unsupported source type. Provide a video file, image file, or image directory."
		)


if __name__ == "__main__":
	main()
