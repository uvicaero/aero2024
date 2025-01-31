import os
import cv2
import numpy as np
from src.functions.detect_hotspots import detect_hotspots

def apply_circular_mask(image):
    """Applies a circular mask to the given image."""
    height, width = image.shape
    mask = np.zeros((height, width), dtype=np.uint8)
    center = (width // 2, height // 2)
    radius = int((3/4) * (width / 2))
    cv2.circle(mask, center, radius, 255, -1)
    return cv2.bitwise_and(image, image, mask=mask)

def detect_hotspots_with_mask(image, threshold=0.7):
    """Applies a circular mask and runs detect_hotspots with a given threshold."""
    masked_image = apply_circular_mask(image)
    return detect_hotspots(masked_image, threshold=threshold)

if __name__ == "__main__":
    print("This module provides functions for detecting hotspots with a circular mask.")
