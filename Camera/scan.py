#!/usr/bin/env python3
"""Decode QR data from an image path and show the result in a small window."""

import argparse
import importlib
import os
import sys
import tkinter as tk
from tkinter import scrolledtext


def decode_with_opencv(image_path):
	try:
		import cv2
	except Exception:
		return None, []

	img = cv2.imread(image_path)
	if img is None:
		return None, []

	detector = cv2.QRCodeDetector()
	text, _, _ = detector.detectAndDecode(img)
	if not text:
		text = None

	aruco_hits = []
	try:
		aruco = cv2.aruco
		for name in dir(aruco):
			if not name.startswith("DICT_"):
				continue
			dict_id = getattr(aruco, name)
			if not isinstance(dict_id, int):
				continue

			try:
				dictionary = aruco.getPredefinedDictionary(dict_id)
			except Exception:
				continue

			try:
				params = aruco.DetectorParameters()
				detector_obj = aruco.ArucoDetector(dictionary, params)
				corners, ids, _ = detector_obj.detectMarkers(img)
			except Exception:
				corners, ids, _ = aruco.detectMarkers(img, dictionary)

			if ids is None:
				continue

			for marker_id in ids.flatten().tolist():
				aruco_hits.append((name, int(marker_id)))
	except Exception:
		# aruco module may not be available in the installed OpenCV build
		pass

	return text, aruco_hits


def decode_with_pyzbar(image_path):
	try:
		pyzbar_module = importlib.import_module("pyzbar.pyzbar")
		pil_image_module = importlib.import_module("PIL.Image")
	except Exception:
		return None

	try:
		decoded = pyzbar_module.decode(pil_image_module.open(image_path))
	except Exception:
		return None

	if not decoded:
		return None

	values = []
	for item in decoded:
		try:
			values.append(item.data.decode("utf-8", errors="replace"))
		except Exception:
			values.append(str(item.data))
	return "\n".join(values)


def decode_codes(image_path):
	qr_text, aruco_hits = decode_with_opencv(image_path)
	if not qr_text:
		qr_text = decode_with_pyzbar(image_path)

	# Deduplicate marker results (same dict/id can be detected multiple times)
	seen = set()
	unique_hits = []
	for dict_name, marker_id in aruco_hits:
		key = (dict_name, marker_id)
		if key in seen:
			continue
		seen.add(key)
		unique_hits.append(key)

	return qr_text, unique_hits


def build_output_text(qr_text, aruco_hits):
	parts = []
	if qr_text:
		parts.append("QR CODE:\n" + qr_text)

	if aruco_hits:
		lines = [f"- {dict_name} | id={marker_id}" for dict_name, marker_id in aruco_hits]
		parts.append("ARUCO MARKERS:\n" + "\n".join(lines))

	if not parts:
		return None

	return "\n\n".join(parts)


def show_result_window(title, content):
	root = tk.Tk()
	root.title(title)
	root.geometry("560x260")

	frame = tk.Frame(root, padx=10, pady=10)
	frame.pack(fill=tk.BOTH, expand=True)

	lbl = tk.Label(frame, text=title, font=("Courier New", 11, "bold"), anchor="w")
	lbl.pack(fill=tk.X, pady=(0, 6))

	txt = scrolledtext.ScrolledText(frame, wrap=tk.WORD, height=10)
	txt.pack(fill=tk.BOTH, expand=True)
	txt.insert(tk.END, content)
	txt.configure(state=tk.DISABLED)

	btn = tk.Button(frame, text="Close", command=root.destroy)
	btn.pack(anchor="e", pady=(8, 0))

	root.mainloop()


def main():
	parser = argparse.ArgumentParser(description="Decode QR from an image")
	parser.add_argument("image_path", help="Path to the image file")
	args = parser.parse_args()

	image_path = os.path.abspath(args.image_path)
	if not os.path.isfile(image_path):
		show_result_window("QR Scan", f"Image not found:\n{image_path}")
		return 1

	qr_text, aruco_hits = decode_codes(image_path)
	result = build_output_text(qr_text, aruco_hits)
	if result:
		show_result_window("QR / ArUco Decoded", result)
		return 0

	show_result_window("QR / ArUco Scan", "No QR code or ArUco marker detected in the selected frame.")
	return 2


if __name__ == "__main__":
	sys.exit(main())
