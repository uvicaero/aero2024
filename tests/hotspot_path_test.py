import time
import datetime
import RPi.GPIO as GPIO
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import threading
import subprocess
import simplekml
from shapely.geometry import Point, Polygon, LineString
from src.functions.path_generator import generate_path_custom_boundary, generate_path_custom_boundary_custom_radii
import os
import math
import numpy as np
import cv2
import xml.etree.ElementTree as ET
from src.functions.prototypes.emitter_detection_prototypes.detect_hotspots_with_mask import detect_hotspots_with_mask
from src.functions.get_hotspots_gps import get_hotspots_gps
from src.functions.upload_kml import upload_kml
from picamera2 import Picamera2
from src.functions.detect_hotspots import detect_hotspots
from src.functions.get_hotspots_gps import get_hotspots_gps
import matplotlib.pyplot as plt
import argparse


def send_heartbeat(the_connection):
    while True:
        the_connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,  # Companion computer type
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,        # Not an autopilot
            0,                                            # Base mode (not relevant for companion)
            0,                                            # Custom mode (not relevant for companion)
            0                                             # System status (e.g., MAV_STATE_ACTIVE)
        )
        time.sleep(1)  # Send heartbeat every second

latest_gps = None

latest_attitude = None  # Stores latest attitude data (pitch, roll, yaw)

def mavlink_listener(the_connection):
    global latest_gps, latest_attitude, latest_local
    toggle_state = 0  # 0: GPS, 1: Attitude, 2: Local Position

    while True:
        try:
            if not the_connection or not the_connection.port:
                time.sleep(2)
                continue

            if toggle_state == 0:
                msg_type = "GLOBAL_POSITION_INT"
            elif toggle_state == 1:
                msg_type = "ATTITUDE"
            else:
                msg_type = "LOCAL_POSITION_NED"

            msg = the_connection.recv_match(type=msg_type, blocking=True, timeout=5)

            if msg:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                if msg_type == "GLOBAL_POSITION_INT":
                    latest_gps = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3)

                elif msg_type == "ATTITUDE":
                    latest_attitude = (msg.pitch, msg.roll, msg.yaw)

                elif msg_type == "LOCAL_POSITION_NED":
                    latest_local = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

                toggle_state = (toggle_state + 1) % 3  # Cycle through states

            else:
                print(f"[WARNING] No {msg_type} data received within timeout (5s). Retrying...")

        except Exception as e:
            print(f"[ERROR] Exception in MAVLink Listener: {e}")
            time.sleep(2)


# Connect to the MAVLink device via USB (replace with your actual port)
the_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for the first heartbeat to confirm the connection
print("Waiting for heartbeat...")
the_connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# Start the heartbeat thread
heartbeat_thread = threading.Thread(target=send_heartbeat, args=(the_connection,), daemon=True)
heartbeat_thread.start()

mav_thread = threading.Thread(target=mavlink_listener, args=(the_connection,), daemon=True)
mav_thread.start()


# Pin configuration
GPIO_PIN = 18

# Setup
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
GPIO.setup(GPIO_PIN, GPIO.IN)


def retrieve_gps():
    """
    Retrieves the most recent GPS and Attitude data.
    Returns (lat, lon, rel_alt, pitch, yaw).
    """
    global latest_gps, latest_attitude

    lat, lon, rel_alt = latest_gps if latest_gps else (None, None, None)

    # Extract only pitch and yaw (ignore roll)
    if latest_attitude:
        pitch, _, yaw = latest_attitude  # Ignore roll using `_`
    else:
        pitch, yaw = None, None  # Default values if attitude is missing

    return lat, lon, rel_alt, pitch, yaw

def retrieve_local():
    """
    Retrieves the most recent local position, velocity, and yaw.
    Returns (x, y, z, vx, vy, vz, yaw).
    """
    global latest_local, latest_attitude

    # Extract latest local position and velocity (x, y, z, vx, vy, vz)
    if latest_local:
        x, y, z, vx, vy, vz = latest_local
    else:
        x, y, z, vx, vy, vz = None, None, None, None, None, None

    # Extract yaw from latest attitude (ignore pitch and roll)
    if latest_attitude:
        _, _, yaw = latest_attitude  # Ignore pitch and roll
    else:
        yaw = None  # Default value if attitude is missing

    return x, y, z, vx, vy, vz, yaw


def wait_for_ack(command):
    """Wait for COMMAND_ACK and verify the result"""
    print(f"Waiting for ACK for command: {command}")
    while True:
        ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack is not None:
            if ack.command == command:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"ACK received for command: {command} (ACCEPTED)")
                    return True
                else:
                    print(f"ACK received for command: {command} (Result: {ack.result})")
                    return False
        else:
            print(f"Timeout waiting for ACK for command: {command}")
            return False

def wait_for_takeoff_completion(target_altitude):
    """
    Wait until the drone reaches the target altitude during takeoff.
    :param target_altitude: The target altitude in meters.
    """
    print(f"Waiting for the drone to reach {target_altitude} meters altitude...")
    while True:
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=50)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # Convert from millimeters to meters
            print(f"Current altitude (GLOBAL_POSITION_INT): {current_alt:.2f} meters")
            if current_alt >= target_altitude * 0.95:
                print(f"Target altitude {target_altitude} meters reached.")
                break
        else:
            print("Timeout waiting for altitude data.")
            return False
        time.sleep(1)
    return True

def format_waypoints_to_dict(waypoints):
    """
    Formats a list of waypoint tuples into a dictionary where each key is a waypoint name 
    and the value is a dictionary with 'latitude', 'longitude', and 'altitude'.

    Args:
        waypoints (list of tuples): Each tuple should be in the form (latitude, longitude, altitude).

    Returns:
        dict: A dictionary with waypoint names as keys and coordinate dictionaries as values.
    """
    waypoint_dict = {}
    for index, (lat, lon, altitude) in enumerate(waypoints, start=1):
        waypoint_name = f"Waypoint {index}"
        waypoint_dict[waypoint_name] = {
            'lat': lat,
            'lon': lon,
            'alt': altitude
        }
    return waypoint_dict
    
def arm_throttle(master):
    """
    Arm the drone's throttle.
    
    Args:
        master: MAVLink connection object.
    """
    print("Arming throttle...")
    master.mav.command_long_send(
        master.target_system,            # Target system (e.g., drone)
        master.target_component,         # Target component (e.g., autopilot)
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command to arm/disarm
        0,                               # Confirmation
        1,                               # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0                # Unused parameters
    )

    # Wait for the drone to be armed
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if not msg:
            continue
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Throttle armed.")
            break

def extract_coordinates_to_dict(subdir, filename):

    """
    Extracts the names and coordinates of markers from a KML file into a dictionary for easy lookup.
    
    Args:
        subdir (str): Relative path to the subdirectory containing the KML file.
        filename (str): Name of the KML file.
    
    Returns:
        dict: A dictionary where each key is the placemark name, and the value is another dictionary with:
              - 'latitude': Latitude of the placemark
              - 'longitude': Longitude of the placemark
              - 'altitude': Altitude of the placemark (if present, default is 0)
    """
    # Construct the full path to the KML file
    filepath = os.path.join(subdir, filename)
    
    # Ensure the file exists
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"The file '{filepath}' does not exist.")
    
    # Parse the KML file
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
    with open(filepath, 'r', encoding='utf-8') as file:
        kml_content = file.read()
    root = ET.fromstring(kml_content)
    
    # Find all Placemark elements
    placemarks = root.findall('.//kml:Placemark', namespace)
    coordinates_dict = {}
    
    for placemark in placemarks:
        # Extract the name
        name = placemark.find('kml:name', namespace).text
        
        # Extract the coordinates
        coordinates = placemark.find('.//kml:coordinates', namespace).text.strip()
        lon, lat, *altitude = map(float, coordinates.split(','))
        
        # Add to the dictionary
        coordinates_dict[name] = {
            'lat': lat,
            'lon': lon,
            'alt': altitude[0] if altitude else 0.0  # Default altitude is 0 if not provided
        }
    
    return coordinates_dict

def send_set_position_target_global_int(connection, latitude, longitude, altitude, coordinate_frame=6):
    """
    Sends a SET_POSITION_TARGET_GLOBAL_INT command to reposition the vehicle.

    Args:
        connection (mavutil.mavlink_connection): MAVLink connection object.
        latitude (float): Target latitude in degrees.
        longitude (float): Target longitude in degrees.
        altitude (float): Target altitude in meters.
        coordinate_frame (int): MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (default: 6).
    """
    connection.mav.set_position_target_global_int_send(
        connection.target_system,  # Target system
        connection.target_component,  # Target component
        0,  # Time boot ms (not used)
        coordinate_frame,  # Coordinate frame (relative altitude)
        0b0000100111111000,  # Type mask (ignore velocity, acceleration, and yaw)
        int(latitude * 1e7),  # Latitude in 1E7 degrees
        int(longitude * 1e7),  # Longitude in 1E7 degrees
        altitude,  # Altitude in meters
        0, 0, 0,  # Velocity (not used)
        0, 0, 0,  # Acceleration (not used)
        0, 0  # Yaw and yaw rate (not used)
    )
    print(f"Set position target global int sent to ({latitude}, {longitude}, {altitude})")

def wait_for_position_target(connection, target_lat, target_lon, target_alt, threshold=0.5):
    """
    Waits for the drone to reach the target position within a given threshold.
    
    Args:
        connection: MAVLink connection object.
        target_lat: Target latitude in degrees.
        target_lon: Target longitude in degrees.
        target_alt: Target altitude in meters.
        threshold: Allowed error margin in meters.
    """
    print("Waiting for the drone to reach the target position...")
    
    earth_radius = 6371000  # Earth's radius in meters
    
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            # Convert current position from scaled integers to degrees
            current_lat_deg = msg.lat / 1e7
            current_lon_deg = msg.lon / 1e7
            
            # Compute the horizontal distance using the haversine formula
            dlat = math.radians(target_lat - current_lat_deg)
            dlon = math.radians(target_lon - current_lon_deg)
            a = (math.sin(dlat / 2) ** 2 +
                 math.cos(math.radians(current_lat_deg)) *
                 math.cos(math.radians(target_lat)) *
                 math.sin(dlon / 2) ** 2)
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            horizontal_distance = earth_radius * c
            
            # Compute altitude difference (the drone reports relative_alt in millimeters)
            current_alt = msg.relative_alt * 0.001  # Convert mm to meters
            alt_diff = abs(current_alt - target_alt)
            
            print(f"Horizontal distance: {horizontal_distance:.2f} m, Altitude difference: {alt_diff:.2f} m")
            
            if horizontal_distance < threshold and alt_diff < threshold:
                print("Target position reached.")
                break
        
        time.sleep(0.5)  # Reduce CPU usage


def takeoff(connection, target_alt):
   
    # Takeoff command
    print(f"Taking off to {target_alt} meters...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_alt
    )
    
    # Wait until the drone reaches target altitude
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_alt = msg.relative_alt * 0.001  # Convert mm to meters
            if current_alt >= target_alt * 0.95:  # Consider 95% as reaching altitude
                print("Target altitude reached.")
                break
        time.sleep(0.5)

def set_mode_rtl(connection):
    """
    Sets the flight mode to RTL (Return-to-Launch) using the MAVLink command.
    
    Note:
        The custom mode value used in this command is '6' as provided in your snippet,
        which is commented as "Custom mode for GUIDED". If you intend to set RTL mode,
        you might need to adjust this value according to your flight controller's specifications.
    
    Args:
        connection: The MAVLink connection object.
    """
    # Send the command to change the mode.
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,      # Confirmation (0 for no confirmation)
        1,      # Base mode (this value is often fixed; check your docs)
        6,      # Custom mode (set to 6 as per your snippet; adjust if needed for RTL)
        0, 0, 0, 0, 0  # Unused parameters
    )
    
    # Wait for an acknowledgement that the mode has been set.
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    
def wait_for_switch():
    # Read the initial state of the GPIO pin
    initial_state = GPIO.input(GPIO_PIN)
    print(f"Waiting for GPIO 18 to change state from {initial_state}...")
    
    # Loop until the current state differs from the initial state
    while True:
        current_state = GPIO.input(GPIO_PIN)
        if current_state != initial_state:
            print(f"GPIO 18 changed state to {current_state}")
            return
        time.sleep(0.5)  # Adjust polling rate as needed

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculates the great-circle distance between two GPS coordinates using the Haversine formula.
    Returns distance in meters.
    """
    R = 6371000  # Radius of Earth in meters

    # Convert latitude and longitude from degrees to radians
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Distance in meters

def calculate_gps_distances(landmark_points, get_gps_points):
    """
    Iterates through both dictionaries **in order** and compares each ordered pair sequentially.
    
    - `landmark_points`: Dict with known landmarks and their GPS coordinates.
    - `get_gps_points`: Dict with real-time GPS data.
    """
    print("\n? GPS Distance Calculations:")

    # Convert both dictionaries to **ordered lists** of their values
    landmark_list = list(landmark_points.values())  # Extract coordinate dictionaries
    gps_list = list(get_gps_points.values())  # Extract GPS coordinate dictionaries

    # Determine the minimum length to avoid index errors
    num_pairs = min(len(landmark_list), len(gps_list))

    # Iterate **through each corresponding ordered pair**
    for i in range(num_pairs):
        lat1, lon1 = landmark_list[i]['lat'], landmark_list[i]['lon']
        lat2, lon2 = gps_list[i]['lat'], gps_list[i]['lon']
        
        distance = haversine(lat1, lon1, lat2, lon2)
        print(f"? Pair {i+1}: {distance:.2f} meters")

    # Handle cases where lists are of **unequal length**
    if len(landmark_list) > num_pairs:
        print(f"?? {len(landmark_list) - num_pairs} extra landmarks not compared.")
    elif len(gps_list) > num_pairs:
        print(f"?? {len(gps_list) - num_pairs} extra GPS points not compared.")

def reposition_drone_over_hotspot(connection, camera, threshold=0.5):
    """
    Grabs location based on image,
    call reposition function untill within acceptable distance from hotspot
    """
    while True:
        # Capture image from camera
        image = camera.capture_array()

        x_offset, y_offset, z_offset = get_offset(connection=connection, image=image)

        if x_offset is None or y_offset is None:
            print("Error retreiving offset")
            return False
        
        distance = math.sqrt(x_offset**2 + y_offset**2)
        print(f"Distance to target: {distance:.2f} meters")

        if distance < threshold:
            return True
        
        current_x, current_y, current_z, _, _, _, yaw = retrieve_local()
        
        if current_x is None or current_y is None or current_z is None:
            print("Error retrieving local position.")
            return False

        # Convert yaw to radians
        yaw_rad = float(yaw)

        # Rotate offsets from body frame to NED frame
        target_x = current_x + (x_offset * math.cos(yaw_rad) - y_offset * math.sin(yaw_rad))
        target_y = current_y + (x_offset * math.sin(yaw_rad) + y_offset * math.cos(yaw_rad))
        target_z = current_z + z_offset
        
        # Send reposition command in body-relative frame
        send_body_offset_local_position(connection, x_offset, y_offset, z_offset)
        
        wait_for_position_target_local(connection, target_x, target_y, target_z)

def wait_for_position_target_local(connection, target_x, target_y, target_z, threshold=0.5,interval=0.5, speed_threshold=0.05):
    #Do Stuff to check current local position and compare it to target
    while True:

        updated_x, updated_y, updated_z, vx, vy, vz, _ = retrieve_local()
        speed = math.sqrt(vx**2 + vy**2 + vz**2)

        x_distance = abs(updated_x - target_x)
        y_distance = abs(updated_y - target_y)
        z_distance = abs(updated_z - target_z)

        distance = math.sqrt(x_distance**2 + y_distance**2 + z_distance**2)
        print(f"[DEBUG] Distance to target: {distance:.2f}m, Speed: {speed:.2f}m/s")

        # Check if the drone is within the threshold and moving slowly enough
        if distance < threshold and speed < speed_threshold:
            print("[SUCCESS] Target position reached.")
            return True
        
        time.sleep(interval)

def get_offset(connection, image, fov_x=62.2, fov_y=48.8, image_width=3280, image_height=2464):
    """
    Detects a hotspot in the image, estimates its offset from the drone
    """
    # Get altitude relative to ground (AGL)
    lat, lon, rel_alt, pitch, azimuth = retrieve_gps()
    if rel_alt is None:
        print("Failed to retrieve relative altitude.")
        return None, None, 0
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    # Detect hotspots
    hotspots = detect_hotspots(gray_image, threshold=0.9)
    if hotspots is None or len(hotspots) == 0:
        print("No hotspots detected.")
        return None, None, 0 
    
    # Assume the largest detected hotspot is the target
    target_hotspot = hotspots[0]  # Selecting the first hotspot detected
    print(f"Detected hotspot at: {target_hotspot}")
    
    # Convert image coordinates to movement offsets
    img_center_x = image_width / 2
    img_center_y = image_height / 2
    
    # Compute meters per pixel scale dynamically based on altitude
    fov_x_rad = math.radians(fov_x)
    fov_y_rad = math.radians(fov_y)
    meters_per_pixel_x = (2 * rel_alt * math.tan(fov_x_rad / 2)) / image_width
    meters_per_pixel_y = (2 * rel_alt * math.tan(fov_y_rad / 2)) / image_height
    
    x_offset = -(target_hotspot[1] - img_center_y) * meters_per_pixel_y  # Forward/Backward (NED X)
    y_offset = (target_hotspot[0] - img_center_x) * meters_per_pixel_x  # Left/Right (NED Y)
    z_offset = 0

    print(f"Offset to hotspot: {x_offset:.2f}m forward/backward, {y_offset:.2f}m right/left")

    return x_offset, y_offset, z_offset

def send_body_offset_local_position(connection, x_offset, y_offset, z_offset):
    """
    Sends a SET_POSITION_TARGET_LOCAL_NED command to reposition the vehicle.
    """
    time_boot_ms = int(round(time.time() * 1000)) & 0xFFFFFFFF
    
    # Ensure offsets are floats
    x_offset = float(x_offset)
    y_offset = float(y_offset)
    z_offset = float(z_offset)
    
    connection.mav.set_position_target_local_ned_send(
        time_boot_ms,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0xDF8,
        x_offset,
        y_offset,
        z_offset,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    print(f"Sent reposition command: x={x_offset}, y={y_offset}, z={z_offset}")

def send_body_offset_local_position(connection, x_offset, y_offset, z_offset):
    """
    Sends a SET_POSITION_TARGET_LOCAL_NED command with the MAV_FRAME_BODY_OFFSET_NED frame.
    
    This command moves the vehicle relative to its current orientation:
      - x_offset: forward/backward offset in meters (positive is forward)
      - y_offset: right/left offset in meters (positive is right)
      - z_offset: down/up offset in meters (positive is down)
    
    Args:
        connection (mavutil.mavlink_connection): The MAVLink connection object.
        x_offset (float): Desired offset in the vehicle's forward direction (meters).
        y_offset (float): Desired offset in the vehicle's right direction (meters).
        z_offset (float): Desired offset in the vehicle's down direction (meters).
    """
    # Bitmask to ignore velocity (bits 3,4,5), acceleration (bits 6,7,8),
    # yaw (bit 10), and yaw rate (bit 11) fields.
    type_mask = 0xDF8  # (in decimal, 3576)

    # Get current time in milliseconds (wraps at 2^32)
    time_boot_ms = int(round(time.time() * 1000)) & 0xFFFFFFFF

    # IMPORTANT: Note the argument order below matches the expected order:
    # time_boot_ms, target_system, target_component, coordinate_frame, type_mask, ...
    connection.mav.set_position_target_local_ned_send(
        time_boot_ms,                                   # time_boot_ms (uint32_t)
        connection.target_system,                       # target_system (uint8_t)
        connection.target_component,                    # target_component (uint8_t)
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,        # coordinate frame (uint8_t)
        type_mask,                                      # type_mask (uint16_t)
        x_offset,                                       # x position (meters)
        y_offset,                                       # y position (meters)
        z_offset,                                       # z position (meters)
        0, 0, 0,                                        # vx, vy, vz (ignored)
        0, 0, 0,                                        # afx, afy, afz (ignored)
        0, 0                                            # yaw, yaw_rate (ignored)
    )
    print(f"Body offset position command sent: x={x_offset}, y={y_offset}, z={z_offset}")


def distance_between_gps(lat1, lon1, lat2, lon2):
    """Calculate approximate distance in meters between two GPS coordinates."""
    R = 6371000  # radius of Earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def imageToHotspotCoordinates(image):
    """
    Gets list of hotspot lat/lon from an image

    Parameters:
        image: either static image from a file or taken live from picam2
    Return:
        detected_hotspots: a list of every hotspot detected in the image as lat/lon pairs in a 2d array [[lat, lon]]
        get_gps_points: a lat/lon array of where the drone was when the photo was taken
    """
    detected_hotspots = []

    hotspots = detect_hotspots(image, threshold=0.7)
    print(f"Detected hotspots: {hotspots}")
    # Get the latest GPS coordinates from drone
    lat, lon, alt, _, yaw = retrieve_gps() ############################SHOULD we use get_latest_gps, retrieve_gps or retrieve_local
    #get_gps_points.append({"lat": lat, "lon": lon})   ###### Don't need to implement till flight test

    # Map hotspots to GPS coordinates using `get_hotspots_gps`
    if hotspots.size > 0:
        distorted_points = [[x, y] for x, y in hotspots]
        dist_pts = np.array(distorted_points, dtype=np.float32)
        pitch = math.radians(-90)
        azimuth = yaw  # Replace with actual azimuth reading
        gps_hotspots = get_hotspots_gps(dist_pts, lon, lat, alt, pitch, azimuth)

        # Fix shape if necessary: Convert (N, 1, 2) ? (N, 2)
        if gps_hotspots.ndim == 3 and gps_hotspots.shape[1] == 1 and gps_hotspots.shape[2] == 2:
            gps_hotspots = gps_hotspots.reshape(-1, 2)

        # Ensure valid data before appending
        if gps_hotspots.size > 0 and gps_hotspots.shape[1] == 2:
            for gps_point in gps_hotspots:
                detected_hotspots.append([gps_point[0], gps_point[1]])
        else:
            print(f"Error: Invalid GPS hotspots output {gps_hotspots}")

    return detected_hotspots

def wait_until_reached(connection, target_lat, target_lon, target_alt, tolerance_m=2.0, alt_tolerance=0.5, velocity_threshold=0.2, stable_time=1.0):
    """
    Commands drone to move to a GPS coordinate and waits until it's stable at that point.

    Parameters:
        target_lat (float): Target latitude
        target_lon (float): Target longitude
        target_alt (float): Target altitude
        tolerance_m (float): Horizontal position tolerance in meters
        alt_tolerance (float): Vertical altitude tolerance in meters
        velocity_threshold (float): Velocity threshold (m/s) to determine stability
        stable_time (float): Duration (seconds) drone must be stable at location
    """

    stable_start = None

    while True:
        current_lat, current_lon, current_alt, _, _ = retrieve_gps()
        print(f"Cur Lat: {current_lat} Cur Lon: {current_lon} Cur Alt: {current_alt}")

        horizontal_distance = distance_between_gps(current_lat, current_lon, target_lat, target_lon)
        vertical_distance = abs(current_alt - target_alt)

        if horizontal_distance <= tolerance_m and vertical_distance <= alt_tolerance:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start >= stable_time:
                print("Drone has stabilized at the target coordinates.")
                break
        else:
            stable_start = None  # Reset stable time if it drifts or moves

        time.sleep(0.5)  # Polling interval

def point_north(connection):
    time_boot_ms = int(round(time.time() * 1000)) & 0xFFFFFFFF
    connection.mav.set_position_target_local_ned_send(
        time_boot_ms,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b100111111000,
        0,
        0,
        0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    time.sleep(5)
    

# ——— Hard-coded offsets for the 18 rectangular FOV centers ———
# Format: (box_index, x_offset_m_east, y_offset_m_north)
PATTERN_OFFSETS = [
    (1,  -27.05969812966711,   0.0),
    (2,  -27.05969812966711,  40.90399071236321),
    (3,   27.05969812966711,  40.90399071236321),
    (4,   27.05969812966711,   0.0),
    (5,   27.05969812966711, -40.90399071315330),
    (6,  -27.05969812966711, -40.90399071315330),
    (7,  -81.17909439319548, -40.90399071315330),
    (8,  -81.17909439319548,   0.0),
    (9,  -81.17909439319548,  40.90399071236321),
    (10, -54.11939626352837,  81.80798142472642),
    (11,   0.0,               81.80798142472642),
    (12,  54.11939626352837,  81.80798142472642),
    (13,  81.17909439319548,  40.90399071236321),
    (14,  81.17909439319548,   0.0),
    (15,  81.17909439319548, -40.90399071315330),
    (16,  54.11939626352837, -81.80798142472642),
    (17,   0.0,              -81.80798142472642),
    (18, -54.11939626352837, -81.80798142472642),
]

def xy_to_latlon(x_m: float, y_m: float, lat0: float, lon0: float) -> (float, float):
    """
    Convert local east/north offsets (meters) to latitude/longitude
    around reference point (lat0, lon0).
    """
    R = 6371000.0  # Earth radius in meters
    dlat = y_m / R
    dlon = x_m / (R * math.cos(math.radians(lat0)))
    lat = lat0 + (dlat * 180.0 / math.pi)
    lon = lon0 + (dlon * 180.0 / math.pi)
    return lat, lon

def get_rectangle_centers_from_list(center_lat: float,
                                    center_lon: float,
                                    pattern_offsets=PATTERN_OFFSETS) -> list:
    """
    Returns a list of (lat, lon, alt) for each rectangular FOV center,
    ordered by the box index in the spiral pattern.
    """
    # Sort by box index to enforce spiral order
    sorted_offsets = sorted(pattern_offsets, key=lambda t: t[0])
    waypoints = []
    for idx, x_m, y_m in sorted_offsets:
        lat, lon = xy_to_latlon(x_m, y_m, center_lat, center_lon)
        waypoints.append((lat, lon))
    return waypoints

def validate_spiral_path(
    waypoint_list,                # [(lat, lon, alt), ...]
    boundary_polygon,             # shapely.geometry.Polygon (lon, lat order)
    cornerfix,                    # (lat, lon)
    altitude=50.0                 # Altitude to use for cornerfix
):
    """
    Given a list of waypoints, return a safe path that drops out-of-bound points
    and inserts `cornerfix` detours where path segments cross the boundary.

    Parameters:
        waypoint_list: ordered list of (lat, lon)
        boundary_polygon: Shapely Polygon in (lon, lat) format
        cornerfix: (lat, lon) tuple to insert when a segment crosses outside
        altitude: altitude to use for cornerfix

    Returns:
        Filtered and patched list of (lat, lon) waypoints
    """
    # Step 1: Filter out-of-bounds points
    valid_points = [
        (lat, lon)
        for lat, lon in waypoint_list
        if boundary_polygon.contains(Point(lon, lat))
    ]

    # Step 2: Insert cornerfix detour where segment crosses outside
    path = []
    for i in range(len(valid_points) - 1):
        lat1, lon1 = valid_points[i]
        lat2, lon2 = valid_points[i + 1]
        segment = LineString([(lon1, lat1), (lon2, lat2)])
        path.append((lat1, lon1))
        if not boundary_polygon.contains(segment):
            path.append((cornerfix[0], cornerfix[1]))

    if valid_points:
        path.append(valid_points[-1])  # Add the last point

    return path

def imageToHotspotCoordinates(image):
    """
    Gets list of hotspot lat/lon from an image

    Parameters:
        image: either static image from a file or taken live from picam2
    Return:
        detected_hotspots: a list of every hotspot detected in the image as lat/lon pairs in a 2d array [[lat, lon]]
        get_gps_points: a lat/lon array of where the drone was when the photo was taken
    """
    detected_hotspots = []

    hotspots = detect_hotspots_with_mask(image, threshold=0.7)
    print(f"Detected hotspots: {hotspots}")
    # Get the latest GPS coordinates from drone
    lat, lon, alt, _, _, _, _, yaw = retrieve_gps() ############################SHOULD we use get_latest_gps, retrieve_gps or retrieve_local
    #get_gps_points.append({"lat": lat, "lon": lon})   ###### Don't need to implement till flight test

    # Map hotspots to GPS coordinates using `get_hotspots_gps`
    if hotspots.size > 0:
        distorted_points = [[x, y] for x, y in hotspots]
        dist_pts = np.array(distorted_points, dtype=np.float32)
        pitch = math.radians(-90)
        azimuth = yaw  # Replace with actual azimuth reading
        gps_hotspots = get_hotspots_gps(dist_pts, lon, lat, alt, pitch, azimuth)

        # Fix shape if necessary: Convert (N, 1, 2) ? (N, 2)
        if gps_hotspots.ndim == 3 and gps_hotspots.shape[1] == 1 and gps_hotspots.shape[2] == 2:
            gps_hotspots = gps_hotspots.reshape(-1, 2)

        # Ensure valid data before appending
        if gps_hotspots.size > 0 and gps_hotspots.shape[1] == 2:
            for gps_point in gps_hotspots:
                detected_hotspots.append([gps_point[0], gps_point[1]])
        else:
            print(f"Error: Invalid GPS hotspots output {gps_hotspots}")

    return detected_hotspots

def generateKML(hotspots):

    # File Save Location
    output_kml_path = "data/kml_source_files/hotspots.kml"

    red_icon = "http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png"

    # Generate KML file

    kml = simplekml.Kml()
    hotspot_count = 1
    for point in hotspots:
        pnt = kml.newpoint(name=f"Hotspot {hotspot_count}", coords=[(point[1], point[0])])  # Lon, Lat
        hotspot_count += 1
        pnt.style.iconstyle.icon.href = red_icon

    # Save KML file
    os.makedirs(os.path.dirname(output_kml_path), exist_ok=True)
    kml.save(output_kml_path)
    print(f"KML file saved to {output_kml_path}")

    # Upload KML file to Google Drive
    upload_kml(output_kml_path, 'https://drive.google.com/drive/folders/1Nc0sSJF1-gshAaj4k81v2x1kxkqtnhiA')
    print(f"KML file uploaded to Google Drive")

# Define the boundary coordinates 
comp_boundary_coords = [
    (50.0971537, -110.7329257),
    (50.1060519, -110.7328869),
    (50.1060793, -110.7436756),
    (50.1035452, -110.7436555),
    (50.0989139, -110.7381534),
    (50.0971788, -110.7381487),
    (50.0971537, -110.7329257)
]

vantreight_boundary_coords = [

    (48.4932345, -123.3099802),
    (48.4930248, -123.3088322),
    (48.4926604, -123.3091166),
    (48.4928844, -123.310176)

]

# Create the boundary polygon
comp_boundary_polygon = Polygon([(lon, lat) for lat, lon in comp_boundary_coords])

vantreight_boundary_polygon = Polygon([(lon, lat) for lat, lon in vantreight_boundary_coords])


cornerfix_comp = (50.0989139, -110.7381534)
cornerfix_vantreight = (48.49254, -123.30896)

def main(boundary_choice):
    if boundary_choice == "comp":
        boundary_polygon = comp_boundary_polygon
        cornerfix = cornerfix_comp
    elif boundary_choice == "vantreight":
        boundary_polygon = vantreight_boundary_polygon
        cornerfix = cornerfix_vantreight
    else:
        print(f"Invalid boundary: {boundary_choice}")
        return
    
        # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (3280, 2464)}  # Maximum resolution
    )
    picam2.configure(config)
    picam2.start()

    time.sleep(2)

    initial_hotspots = []
    final_hotspots = []


    while True:
        # Wait to start survey
        user_input = input(f"Press Enter to start") ######## FOR TESTING ################################

        # Orient correctly
        point_north(the_connection)
        lat, lon, _, _, _ = retrieve_gps()

        # Generate waypoints for initial survey and validate
        waypoints = get_rectangle_centers_from_list(lat, lon)
        validated_waypoints = validate_spiral_path(waypoints, boundary_polygon, cornerfix)
        print(validated_waypoints) 

        # First pass, for each waypoint:
        #  1. Go to point
        #  2. Take photo
        #  3. Detect hotspots and store
        for lat, lon in validated_waypoints:
            user_input = input(f"Press Enter to start") ######## FOR TESTING ################################

            # Go to point
            send_set_position_target_global_int(the_connection, lat, lon, 50)
            print(f"Waiting until reached...") 
            wait_until_reached(the_connection, lat, lon, 50)
            print(f"Point reached") 
            time.sleep(1)
            print("\nCapturing image...")

            # Capture image and extract hotspots
            rgb_image = picam2.capture_array("main")
            image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            detected_hotspots = imageToHotspotCoordinates(image)
            initial_hotspots.extend(detected_hotspots)

            
        # Create a valid path to visit detected hotspots (Doesnt cross outside boundary)
        validated_hotspots = validate_spiral_path(initial_hotspots, boundary_polygon, cornerfix)
        
        # Second pass, for each hotspot guess:
        #  1. Go to point
        #  2. Reposition over spot
        #  3. Store final estimate
        for lat, lon in validated_hotspots:
            user_input = input(f"Press Enter to start press enter") ######## FOR TESTING ################################
            # Go to point
            send_set_position_target_global_int(the_connection, lat, lon, 20)
            print(f"Waiting until reached...") 
            wait_until_reached(the_connection, lat, lon, 20)
            print(f"Point reached") 
            time.sleep(1)
            print("\nCapturing image...")
            user_input = input(f"Get ready for photo press enter") ######## FOR TESTING ################################
            # look for hotspot
            print(f"Taking photo...")
            rgb_image = picam2.capture_array("main")
            image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            hotspot = imageToHotspotCoordinates(image)
            
            # try up to 3 times, ascending by 5 m each time
            max_retries = 3
            for attempt in range(1, max_retries+1):
                if hotspot:
                    break
                print(f"No hotspot found on attempt {attempt}, ascending and retrying…")
                cur_lat, cur_lon, _, _, _ = retrieve_gps()
                new_alt = 20 + 5 * attempt
                send_set_position_target_global_int(the_connection, cur_lat, cur_lon, new_alt)
                print("Waiting until reached…")
                wait_until_reached(the_connection, cur_lat, cur_lon, new_alt)
                print("Retake photo…")
                user_input = input(f"Press Enter to start press enter") ######## FOR TESTING ################################
                rgb_image = picam2.capture_array("main")
                image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
                hotspot = imageToHotspotCoordinates(image)

            # if still no hotspot, skip this point
            if not hotspot:
                print(f"[WARN] No hotspot after {max_retries} attempts; skipping this point.")
                continue

            
            
            # reposition within 1m
            threshold = 1  
            user_input = input(f"First reposition press enter") ######## FOR TESTING ################################
            reposition_drone_over_hotspot(the_connection, picam2, threshold)
            # descend to 10m
            cur_lat, cur_lon, _, _, _ = retrieve_gps()
            user_input = input(f"Descend press enter") ######## FOR TESTING ################################
            send_set_position_target_global_int(the_connection, cur_lat, cur_lon, 10 )
            # reposition within 0.5m
            threshold = 0.5  
            user_input = input(f"Second reposition press enter") ######## FOR TESTING ################################
            reposition_drone_over_hotspot(the_connection, picam2, threshold)
            user_input = input(f"Final capture press enter") ######## FOR TESTING ################################
            rgb_image = picam2.capture_array("main")
            image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            hotspot = imageToHotspotCoordinates(image)
            final_hotspots.extend(hotspot)

        # generate and upload kml of final hotspots
        generateKML(final_hotspots)


           

   


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run drone spiral path with selected boundary.")
    parser.add_argument("--boundary", type=str, choices=["comp", "vantreight"], default="comp",
                        help="Boundary to use: 'comp' or 'vantreight'")
    args = parser.parse_args()
    main(args.boundary)
