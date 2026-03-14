"""
Ilmenite-Rich Basalt Detector
ARC Rover Challenge — Shackleton Prospect Task 7

Usage:
    python detect_ilmenite.py image.jpg
    python detect_ilmenite.py image.png

Output:
    Ilmenite Basalt detected
    OR
    Ilmenite Basalt not detected
"""

import sys
import cv2
import numpy as np


# ──────────────────────────────────────────────
# TUNABLE THRESHOLDS  (adjust after field tests)
# ──────────────────────────────────────────────
L_MAX            = 85    # LAB lightness ceiling  (0-255 OpenCV scale)
                         # Ilmenite is very dark; raise if missing it
MIN_AREA         = 300   # minimum contour area in pixels
MIN_SOLIDITY     = 0.45  # convex-hull fill ratio (rejects shadow tendrils)
MIN_TEXTURE      = 15.0  # GLCM contrast floor    (rejects smooth shadows)
DARK_PIXEL_FRAC  = 0.30  # fraction of contour interior that must be dark
CONFIDENCE_VOTES = 2     # how many checks must pass (out of 3) to confirm
# ──────────────────────────────────────────────


def load_image(path: str) -> np.ndarray:
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"Cannot open image: {path}")
    # Downscale very large images to cap processing time
    h, w = img.shape[:2]
    if max(h, w) > 1920:
        scale = 1920 / max(h, w)
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    return img


def build_dark_mask(img: np.ndarray) -> np.ndarray:
    """
    Combine LAB lightness and HSV value to isolate very dark regions.
    Ilmenite basalt: L* ≈ 15-40 (perceptually very dark, near-black).
    Anorthosite / regolith simulants are noticeably lighter (L* > 60).
    """
    # Denoise first — preserves edges, smooths noise
    denoised = cv2.bilateralFilter(img, d=9, sigmaColor=75, sigmaSpace=75)

    # LAB lightness channel
    lab  = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
    L    = lab[:, :, 0]                          # 0-255 in OpenCV
    mask_lab = (L <= L_MAX).astype(np.uint8) * 255

    # HSV value channel (cross-check)
    hsv  = cv2.cvtColor(denoised, cv2.COLOR_BGR2HSV)
    V    = hsv[:, :, 2]
    mask_hsv = (V <= L_MAX).astype(np.uint8) * 255

    # Require both channels to agree → fewer false positives
    combined = cv2.bitwise_and(mask_lab, mask_hsv)

    # Morphological cleanup: close small holes, open speckles
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, k_close)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN,  k_open)

    return combined


def glcm_contrast(patch: np.ndarray) -> float:
    """
    Approximate GLCM contrast using local standard deviation of gradients.
    True GLCM is slow; this is fast, well-correlated, and shadow-resistant.
    Rocks have high local contrast; smooth shadows have near-zero.
    """
    if patch.size == 0:
        return 0.0
    gray  = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY) if len(patch.shape) == 3 else patch
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    magnitude = np.sqrt(sobelx**2 + sobely**2)
    return float(np.std(magnitude))


def check_edge_sharpness(mask_roi: np.ndarray) -> float:
    """
    Sharp, well-defined edges → rock.
    Soft gradient edges → shadow.
    Returns mean edge response along the contour boundary.
    """
    laplacian = cv2.Laplacian(mask_roi, cv2.CV_64F)
    return float(np.mean(np.abs(laplacian)))


def analyse_contour(cnt, img: np.ndarray, mask: np.ndarray) -> dict:
    """
    Run all geometric and texture checks on a single contour.
    Returns a dict with individual test results.
    """
    result = {"area": 0, "solidity": 0, "texture": 0,
              "dark_frac": 0, "passes": 0, "valid": False}

    area = cv2.contourArea(cnt)
    result["area"] = area
    if area < MIN_AREA:
        return result

    # ── Solidity (shape regularity) ──────────────────────────
    hull     = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity  = area / hull_area if hull_area > 0 else 0
    result["solidity"] = solidity

    # ── Bounding box extraction ──────────────────────────────
    x, y, w, h = cv2.boundingRect(cnt)
    # Guard against out-of-bounds
    ih, iw = img.shape[:2]
    x, y   = max(x, 0), max(y, 0)
    x2, y2 = min(x + w, iw), min(y + h, ih)
    if (x2 - x) < 5 or (y2 - y) < 5:
        return result

    patch      = img[y:y2, x:x2]
    mask_patch = mask[y:y2, x:x2]

    # ── Texture contrast ─────────────────────────────────────
    texture = glcm_contrast(patch)
    result["texture"] = texture

    # ── Dark pixel fraction inside contour ───────────────────
    total_pixels = (x2 - x) * (y2 - y)
    dark_pixels  = int(np.sum(mask_patch > 0))
    dark_frac    = dark_pixels / total_pixels if total_pixels > 0 else 0
    result["dark_frac"] = dark_frac

    # ── Vote counting ────────────────────────────────────────
    votes = 0
    if solidity  >= MIN_SOLIDITY:    votes += 1
    if texture   >= MIN_TEXTURE:     votes += 1
    if dark_frac >= DARK_PIXEL_FRAC: votes += 1

    result["passes"] = votes
    result["valid"]  = (votes >= CONFIDENCE_VOTES)
    return result


def detect_ilmenite(path: str) -> bool:
    """
    Main detection function.
    Returns True if ilmenite-rich basalt is detected.
    """
    img  = load_image(path)
    mask = build_dark_mask(img)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        result = analyse_contour(cnt, img, mask)
        if result["valid"]:
            return True

    return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python detect_ilmenite.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]

    try:
        detected = detect_ilmenite(image_path)
    except FileNotFoundError as e:
        print(str(e))
        sys.exit(1)

    if detected:
        print("Ilmenite Basalt detected")
    else:
        print("Ilmenite Basalt not detected")


if __name__ == "__main__":
    main()