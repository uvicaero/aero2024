#!/usr/bin/env python3
"""
test_hotspot_detection.py

Standalone script to test Picamera2 photo capture and hotspot detection.
"""
import time
import argparse
import cv2
import numpy as np
from picamera2 import Picamera2
from src.functions.detect_hotspots import detect_hotspots

def main(resolution, gain, exposure, threshold, preview):
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
    print("Camera initialized ({}×{}) at gain={} and exposure={}µs".format(
        resolution[0], resolution[1], gain, exposure)
    )

    try:
        while True:
            input("Press Enter to capture an image (Ctrl+C to exit)...")
            # Capture and convert to grayscale
            rgb = picam2.capture_array("main")
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

            # Detect hotspots
            hotspots = detect_hotspots(gray, threshold=threshold)
            print(f"Detected hotspots: {hotspots}")

            if preview:
                # Draw markers on the image
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                for y, x in hotspots:
                    cv2.drawMarker(vis, (int(x), int(y)), (0, 0, 255), markerType=cv2.MARKER_CROSS)
                cv2.imshow("Hotspot Preview", vis)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
    except KeyboardInterrupt:
        print("Exiting test script.")
    finally:
        picam2.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test script for Picamera2 photo capture and hotspot detection."
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
        help="Show a preview window with detected hotspots"
    )
    args = parser.parse_args()
    main(
        resolution=args.resolution,
        gain=args.gain,
        exposure=args.exposure,
        threshold=args.threshold,
        preview=args.preview
    )
