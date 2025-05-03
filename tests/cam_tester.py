#!/usr/bin/env python3
"""
test_hotspot_detection.py

Standalone script to test Picamera2 photo capture and hotspot detection,
with matplotlib preview and saving annotated images.
"""
import time
import os
import argparse
import datetime
import cv2
import numpy as np
import matplotlib.pyplot as plt
from picamera2 import Picamera2
from src.functions.detect_hotspots import detect_hotspots

def main(resolution, gain, exposure, threshold, preview):
    # Prepare output directory structure
    base_dir = os.path.join("data", "test_outputs", "cam_test_photos")
    subdir = f"gain_{gain}_exp_{exposure}_thr_{threshold}"
    output_dir = os.path.join(base_dir, subdir)
    os.makedirs(output_dir, exist_ok=True)

    # Initialize Picamera2
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (resolution[0], resolution[1])}
    )
    picam2.configure(config)
    picam2.set_controls({
        "AeEnable": False,
        "AnalogueGain": gain,
        "ExposureTime": exposure
    })
    picam2.start()
    time.sleep(2)  # allow sensor to settle
    print(f"Camera initialized ({resolution[0]}×{resolution[1]}) at gain={gain}, exposure={exposure}µs, threshold={threshold}")

    try:
        while True:
            input("Press Enter to capture an image (Ctrl+C to exit)...")

            # Capture and convert to grayscale
            rgb = picam2.capture_array("main")
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

            # Detect hotspots
            hotspots = detect_hotspots(gray, threshold=threshold)
            print(f"Detected hotspots: {hotspots}")

            # Annotate image
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            fig, ax = plt.subplots()
            ax.imshow(gray, cmap='gray')
            # Unpack correctly: detect_hotspots returns (x, y) pairs
            for x, y in hotspots:
                ax.plot(x, y, marker='x', markersize=10)
            ax.axis('off')

            # Preview and hold until closed
            if preview:
                plt.show()  # Blocks until window is closed by the user

            # Save annotated image
            filename = f"{timestamp}.png"
            out_path = os.path.join(output_dir, filename)
            fig.savefig(out_path, bbox_inches='tight', pad_inches=0)
            plt.close(fig)
            print(f"Saved annotated image to {out_path}")

    except KeyboardInterrupt:
        print("Exiting test script.")
    finally:
        picam2.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test script for Picamera2 capture and hotspot detection with matplotlib preview and saving."
    )
    parser.add_argument(
        "-r", "--resolution", nargs=2, type=int, default=[3280, 2464],
        help="Width and height for still capture (default: 3280 2464)"
    )
    parser.add_argument(
        "-g", "--gain", type=float, default=9.0,
        help="Analogue gain for manual control (e.g., 2.0 for ISO100)"
    )
    parser.add_argument(
        "-e", "--exposure", type=int, default=15000,
        help="Exposure time in microseconds (default: 15000)"
    )
    parser.add_argument(
        "-t", "--threshold", type=float, default=0.7,
        help="Hotspot detection threshold (default: 0.7)"
    )
    parser.add_argument(
        "-p", "--preview", action="store_true",
        help="Show a matplotlib preview that stays open until you close it"
    )
    args = parser.parse_args()
    main(
        resolution=args.resolution,
        gain=args.gain,
        exposure=args.exposure,
        threshold=args.threshold,
        preview=args.preview
    )
