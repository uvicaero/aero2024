#!/usr/bin/env python

import argparse
import os
import time
from datetime import datetime
from picamzero import Camera
import cv2
import sys
from pymavlink import mavutil
from src.functions.mav_utils import get_latest_gps, send_waypoint, send_waypoints, set_auto_mode
from src.functions.path_generator import generate_path


sys.path.append("./utils")
from functions.detect_hotspots import detect_hotspots

def main():
    # Create the MAVLink connection
    connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

    # Get current location
    lat, lon, alt = get_latest_gps(connection)

    # Generate Waypoints
    waypoints = generate_path(lat, lon, alt)

    # Program pixhawk with waypoints
    send_waypoints(connection, waypoints)

    #Start automatic mission
    set_auto_mode(connection)

    # TODO
    #
    # Loop:
    # 
    #   Stop when MAVLink connection says it is done with waypoints
    #
    #   Take a photo every N seconds (3 for now)
    #
    #   Run detect_hotspots() on latest photo
    #
    #   Run get_latest_gps()
    #
    #   Run get_hotspots_gps()
    #
    #   Add any detected hotspots to a list with an index for the photo it was in
    #
    # Function:
    #
    #   Eliminate duplicate hotspots in list
    #   Mark duplicate hotspots that are close together
    #
    #       If they appear in the same index: Treat them as separate hotspots
    #       If they never appear in the same index: Average them into one
    #
    # make_kml with new list
    #
    # upload_kml



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
                timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
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
