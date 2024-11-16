#!/usr/bin/env python

import argparse
import os
import time
from datetime import datetime
from picamzero import Camera
import cv2
import sys

sys.path.append("./utils")
from functions.detect_hotspots import detect_hotspots

def main():
    parser = argparse.ArgumentParser(description="Capture images and detect hotspots.")
    parser.add_argument("--debug", action="store_true", help="Save captured images for debugging.")
    args = parser.parse_args()

    cam = Camera()
    debug = args.debug

    #create the "images" folder if debug mode is enabled
    if debug:
        os.makedirs("images", exist_ok=True)

    try:
        while True:
            #capture image as NumPy array stored in memory
            image = cam.capture_array()  #assuming picamzero supports this
            image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  #convert to one channel

            if debug:
                #save the image in the "images" folder with a timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                image_path = f"images/photo_{timestamp}.jpg"
                cv2.imwrite(image_path, image)
                print(f"Image saved for debugging: {image_path}")

            #pass the in-memory image to detect_hotspots
            hotspots = detect_hotspots(image_gray, threshold=0.9, debug=debug)
            print(f"Detected hotspots: {hotspots}")

            time.sleep(3)
    except KeyboardInterrupt:
        print("Camera stopped.")

if __name__ == "__main__":
    main()
