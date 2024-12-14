#!/usr/bin/env python

import argparse
import os
import time
from datetime import datetime
from picamzero import Camera
import cv2
import sys
from pymavlink import mavutil
from src.functions.mav_utils import get_latest_gps, send_waypoints, set_auto_mode
from src.functions.path_generator import generate_path
from src.functions.detect_hotspots import detect_hotspots
from src.functions.make_kml import make_kml
from src.functions.upload_kml import upload_kml
from src.functions.get_hotspots_gps import get_hotspots_gps

sys.path.append("./utils")

def eliminate_duplicates(hotspots):
    """Eliminate duplicate hotspots."""
    unique_hotspots = []
    for i, hotspot in enumerate(hotspots):
        found = False
        for existing in unique_hotspots:
            if abs(existing["lat"] - hotspot["lat"]) < 0.0001 and abs(existing["lon"] - hotspot["lon"]) < 0.0001:
                found = True
                existing["count"] += 1
                existing["lat"] = (existing["lat"] * (existing["count"] - 1) + hotspot["lat"]) / existing["count"]
                existing["lon"] = (existing["lon"] * (existing["count"] - 1) + hotspot["lon"]) / existing["count"]
                break
        if not found:
            unique_hotspots.append({"lat": hotspot["lat"], "lon": hotspot["lon"], "count": 1, "index": hotspot["index"]})
    return unique_hotspots

def main():
    # Create the MAVLink connection
    connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

    # Get current location
    lat, lon, alt = get_latest_gps(connection)

    # Generate Waypoints
    waypoints = generate_path(lat, lon, alt)

    # Program pixhawk with waypoints
    send_waypoints(connection, waypoints)

    # Start automatic mission
    set_auto_mode(connection)

    parser = argparse.ArgumentParser(description="Capture images and detect hotspots.")
    parser.add_argument("--debug", action="store_true", help="Save captured images for debugging.")
    args = parser.parse_args()

    cam = Camera()
    debug = args.debug

    # Create the "images" folder if debug mode is enabled
    if debug:
        os.makedirs("images", exist_ok=True)

    # Initialize hotspots list
    detected_hotspots = []

    try:
        while True:
            # Check if the mission is complete
            if connection.recv_match(type="MISSION_ITEM_REACHED", blocking=True):
                print("Mission completed.")
                break

            # Capture image as NumPy array stored in memory
            image = cam.capture_array()  # Assuming picamzero supports this
            image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)  # Convert to one channel

            # Save the image in the "images" folder with a timestamp if debugging
            if debug:
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                image_path = f"images/photo_{timestamp}.jpg"
                cv2.imwrite(image_path, image)
                print(f"Image saved for debugging: {image_path}")

            # Detect hotspots in the image
            hotspots = detect_hotspots(image_gray, threshold=0.9, debug=debug)
            print(f"Detected hotspots: {hotspots}")

            # Get the latest GPS coordinates from drone
            lat, lon, alt = get_latest_gps(connection)

            # Map hotspots to GPS coordinates using `get_hotspots_gps`
            if hotspots:
                distorted_points = [[x, y] for x, y in hotspots]
                pitch = 0.0  # Replace with actual pitch reading
                azimuth = 0.0  # Replace with actual azimuth reading
                gps_hotspots = get_hotspots_gps(distorted_points, lon, lat, alt, pitch, azimuth)
                for gps_point in gps_hotspots:
                    detected_hotspots.append({"lat": gps_point[0], "lon": gps_point[1]})

        # Eliminate duplicates and refine hotspots
        refined_hotspots = eliminate_duplicates(detected_hotspots)

        # Generate KML file with refined hotspots
        kml_path = make_kml(refined_hotspots, filename="hotspots.kml")
        print(f"KML file generated: {kml_path}")

        # Upload KML file
        upload_kml(kml_path)
        print("KML file uploaded.")

    except KeyboardInterrupt:
        print("Mission stopped by user.")

if __name__ == "__main__":
    main()
