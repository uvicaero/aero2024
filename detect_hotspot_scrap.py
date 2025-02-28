#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
from picamera2 import Picamera2

# Function to detect hotspots in a grayscale image
def detect_hotspots(image, threshold=0.7):
    # Apply binary thresholding
    thresh_val = round(255 * threshold)
    _, thresholded_image = cv2.threshold(image, thresh_val, 255, cv2.THRESH_BINARY)

    # Connected component analysis
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresholded_image, 8, cv2.CV_32S)

    # Filter out small components (background is index 0)
    areas = stats[1:, cv2.CC_STAT_AREA]
    centroids = centroids[1:]
    filtered = centroids[areas >= 10]  # Keep only components with area ? 10 pixels

    return filtered

# ? **Main function: Captures image from PiCamera2, converts to grayscale, and runs hotspot detection**
def main():
    print("\nCapturing image from PiCamera2...\n")

    # Initialize PiCamera2 **with normal image format (RGB)**
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (640, 480)}  # Normal RGB format
    )
    picam2.configure(config)
    picam2.start()

    # Capture **RGB image** and convert it to grayscale
    rgb_image = picam2.capture_array("main")
    picam2.stop()

    # Convert to grayscale manually
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    print("Running hotspot detection...\n")
    hotspots = detect_hotspots(gray_image, threshold=0.5)

    # Plot the image and detected hotspots
    plt.figure(figsize=(6, 5))
    plt.imshow(gray_image, cmap="gray")
    plt.axis("off")

    if hotspots is not None and len(hotspots) > 0:
        plt.scatter(hotspots[:, 0], hotspots[:, 1], color='red', marker='x', label="Hotspots")

    plt.legend()
    plt.title("Detected Hotspots")
    plt.show()

    print(f"Detected {len(hotspots) if hotspots is not None else 0} hotspots.")

if __name__ == "__main__":
    main()
