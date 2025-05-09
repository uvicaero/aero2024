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
import xml.dom.minidom


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
        _, _, current_alt, _, _ = retrieve_gps()
        
        if current_alt >= target_altitude * 0.95:
            print(f"Target altitude {target_altitude} meters reached.")
            break
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
    #wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    
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
    hotspots = detect_hotspots(gray_image, threshold=0.7)
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
        0b100111111000,
        x_offset,
        y_offset,
        z_offset,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    print(f"Sent reposition command: x={x_offset}, y={y_offset}, z={z_offset}")

# def send_body_offset_local_position(connection, x_offset, y_offset, z_offset):
#     """
#     Sends a SET_POSITION_TARGET_LOCAL_NED command with the MAV_FRAME_BODY_OFFSET_NED frame.
    
#     This command moves the vehicle relative to its current orientation:
#       - x_offset: forward/backward offset in meters (positive is forward)
#       - y_offset: right/left offset in meters (positive is right)
#       - z_offset: down/up offset in meters (positive is down)
    
#     Args:
#         connection (mavutil.mavlink_connection): The MAVLink connection object.
#         x_offset (float): Desired offset in the vehicle's forward direction (meters).
#         y_offset (float): Desired offset in the vehicle's right direction (meters).
#         z_offset (float): Desired offset in the vehicle's down direction (meters).
#     """
#     # Bitmask to ignore velocity (bits 3,4,5), acceleration (bits 6,7,8),
#     # yaw (bit 10), and yaw rate (bit 11) fields.
#     type_mask = 0xDF8  # (in decimal, 3576)

#     # Get current time in milliseconds (wraps at 2^32)
#     time_boot_ms = int(round(time.time() * 1000)) & 0xFFFFFFFF

#     # IMPORTANT: Note the argument order below matches the expected order:
#     # time_boot_ms, target_system, target_component, coordinate_frame, type_mask, ...
#     connection.mav.set_position_target_local_ned_send(
#         time_boot_ms,                                   # time_boot_ms (uint32_t)
#         connection.target_system,                       # target_system (uint8_t)
#         connection.target_component,                    # target_component (uint8_t)
#         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,        # coordinate frame (uint8_t)
#         type_mask,                                      # type_mask (uint16_t)
#         x_offset,                                       # x position (meters)
#         y_offset,                                       # y position (meters)
#         z_offset,                                       # z position (meters)
#         0, 0, 0,                                        # vx, vy, vz (ignored)
#         0, 0, 0,                                        # afx, afy, afz (ignored)
#         0, 0                                            # yaw, yaw_rate (ignored)
#     )
#     print(f"Body offset position command sent: x={x_offset}, y={y_offset}, z={z_offset}")


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

import math

def point_north(connection):
    # 1) get current yaw in degrees 0…360
    _, _, _, _, yaw = retrieve_gps()
    
    current_deg = (math.degrees(yaw) + 360) % 360
    # 2) compute angular difference to 0° (north)
    diff = (0 - current_deg) % 360
    # 3) pick shortest direction
    if diff == 0:
        print("Already pointing north")
        return
    elif diff < 180:
        direction = 1   # CW (turn diff degrees)
        angle     = diff
    else:
        direction = -1  # CCW (turn 360-diff degrees)
        angle     = 360 - diff

    # 4) send the yaw command
    yaw_rate = 10  # deg/s
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,            # confirmation
        0,            # param1: ignored in relative=0 absolute mode
        yaw_rate,     # param2: yaw speed
        direction,    # param3: 1=cw, -1=ccw
        0,            # param4: relative=0 → absolute to param1 (we ignore it, using direction+angle)
        0, 0, 0       # unused
    )
    # wait long enough for the turn to complete
    duration = angle / yaw_rate
    time.sleep(duration + 0.5)
    print(f"Rotated to north via {'CW' if direction==1 else 'CCW'} ({angle:.1f}°)")


def wait_until_pointing_north():
    while True:
        _, _, _, _, yaw = retrieve_gps()
        if(yaw <= 0.1):
            break
        time.sleep(0.5)  # Polling interval


    

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


def generateKML(hotspots, flags, sources=None):
    """
    hotspots: list of (lat, lon)
    flags:    list of "multi-blob" or "refine"
    sources:  list of (lat, lon, description)
    """
    output_kml_path = "data/kml_source_files/hotspots.kml"
    red_icon        = "http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png"
    fire_icon       = "http://maps.google.com/mapfiles/kml/pushpin/fire.png"

    kml = simplekml.Kml()

    # 1) Plot your surveyed hotspots
    for i, ((lat, lon), flag) in enumerate(zip(hotspots, flags), start=1):
        method = "clustered@20m" if flag=="multi-blob" else "refined@10m"
        p = kml.newpoint(
            name=f"Hotspot {i} ({method})",
            coords=[(lon, lat)]
        )
        p.description = (
            "Clustered multiple blobs at 20 m"
            if flag=="multi-blob"
            else "Refined single blob at 10 m"
        )
        p.style.iconstyle.icon.href = red_icon

    # 2) Plot any manual “source of fire” markers
    if sources:
        for j, (lat, lon, desc) in enumerate(sources, start=1):
            s = kml.newpoint(
                name=f"Source of fire: {desc}",
                coords=[(lon, lat)]
            )
            s.description = f"Manually marked source of fire: {desc}"
            s.style.iconstyle.icon.href = fire_icon

    # Save & upload
    os.makedirs(os.path.dirname(output_kml_path), exist_ok=True)
    kml.save(output_kml_path)
    print(f"KML file saved to {output_kml_path}")
    upload_kml(output_kml_path, 'https://drive.google.com/drive/folders/1Nc0sSJF1-gshAaj4k81v2x1kxkqtnhiA')
    print("KML file uploaded to Google Drive")

def save_kml_minimal_format(hotspots, source_marker, output_path="data/kml_source_files/hotspots_minimal.kml"):
    """
    Save minimal KML with hotspots and a source marker.
    Then upload to Google Drive and print confirmation.
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    kml = ET.Element("kml", xmlns="http://www.opengis.net/kml/2.2")
    document = ET.SubElement(kml, "Document")

    # Add source
    if source_marker:
        source_lat, source_lon, desc = source_marker
        placemark = ET.SubElement(document, "Placemark")
        ET.SubElement(placemark, "name").text = "Source"
        ET.SubElement(placemark, "description").text = desc
        point = ET.SubElement(placemark, "Point")
        ET.SubElement(point, "coordinates").text = f"{source_lon},{source_lat},0"

    # Add hotspots
    for idx, (lat, lon) in enumerate(hotspots, 1):
        placemark = ET.SubElement(document, "Placemark")
        ET.SubElement(placemark, "name").text = f"Hotspot {idx}"
        point = ET.SubElement(placemark, "Point")
        ET.SubElement(point, "coordinates").text = f"{lon},{lat},0"

    # Convert to pretty-printed XML string
    rough_string = ET.tostring(kml, encoding='utf-8')
    reparsed = xml.dom.minidom.parseString(rough_string)
    pretty_kml = reparsed.toprettyxml(indent=" ", newl="\n")

    # Write to file
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(pretty_kml)

    print(f"KML file saved to {output_path}")

    # Upload the file
    upload_kml(output_path, 'https://drive.google.com/drive/folders/1Nc0sSJF1-gshAaj4k81v2x1kxkqtnhiA')
    print("KML file uploaded to Google Drive")

    return output_path


def arm_and_takeoff(connection, altitude):
    # Set mode
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        4, 0, 0, 0, 0, 0  # Custom mode for GUIDED
    )
    #wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)

    # Arm the vehicle
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # Arm
        0, 0, 0, 0, 0, 0
    )
    print("Vehicle armed.")

    # Takeoff to an altitude of 10 meters
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, float('nan'),
        0, 0,
        altitude  # Target altitude in meters
    )
    print("Takeoff command sent.")

    #wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

def set_mode_loiter(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,      # 176
        0,                                         # confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # 1
        5,                                         # Loiter mode (custom_mode = 5) :contentReference[oaicite:0]{index=0}
        0, 0, 0, 0, 0                              # unused
    )
    #wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    print("Now in Loiter mode")

def set_mode_guided(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,      # 176
        0,                                         # confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # 1
        4,                                         # Loiter mode (custom_mode = 5) :contentReference[oaicite:0]{index=0}
        0, 0, 0, 0, 0                              # unused
    )
    #wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    print("Now in Guided mode")

def set_mode_RTL(connection):
    # DO_SET_MODE (176), param1=1 to enable custom mode, param2=6 for RTL
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,            # 176
        0,                                               # confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # enable custom mode
        6,                                               # RTL mode (custom_mode = 6)
        0, 0, 0, 0, 0                                    # unused
    )
    #wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    print("Now in RTL mode")

def cluster_hotspots(hotspots, threshold_m=5.0):
    """
    Merge any hotspots closer than threshold_m into a single cluster,
    returning the list of cluster centers.
    """
    clusters = []  # each entry is [sum_lat, sum_lon, count]
    for lat, lon in hotspots:
        placed = False
        for c in clusters:
            # compute distance from this point to cluster center
            avg_lat, avg_lon = c[0]/c[2], c[1]/c[2]
            if distance_between_gps(lat, lon, avg_lat, avg_lon) < threshold_m:
                c[0] += lat
                c[1] += lon
                c[2] += 1
                placed = True
                break
        if not placed:
            clusters.append([lat, lon, 1])
    # turn sums back into centers
    return [(c[0]/c[2], c[1]/c[2]) for c in clusters]

def save_boundary_validated_and_eliminated_kml(
    validated_waypoints, all_waypoints, boundary_polygon, output_path="data/kml_source_files/boundary_validated_eliminated.kml"
):
    """
    Save a KML showing:
      - Validated points (inside boundary)
      - Eliminated points (outside boundary)
      - Boundary polygon
    Then upload it to Google Drive.
    """
    import simplekml
    import os

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    kml = simplekml.Kml()

    # Track which waypoints were validated
    validated_set = set(validated_waypoints)

    # Add ALL waypoints, but colored by validity
    for idx, (lat, lon) in enumerate(all_waypoints, 1):
        pnt = kml.newpoint(coords=[(lon, lat)])
        if (lat, lon) in validated_set:
            pnt.name = f"VALID Waypoint {idx}"
            pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/blu-circle.png"
        else:
            pnt.name = f"ELIMINATED Waypoint {idx}"
            pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/red-circle.png"

    # Add the boundary polygon
    if boundary_polygon:
        coords = [(lon, lat) for lon, lat in boundary_polygon.exterior.coords]
        pol = kml.newpolygon(name="Boundary Region", outerboundaryis=coords)
        pol.style.polystyle.color = "7d00ff00"  # semi-transparent green
        pol.style.linestyle.width = 2

    # Save KML file
    kml.save(output_path)
    print(f"Boundary and waypoints (validated + eliminated) KML saved to {output_path}")

    # Upload to Drive
    upload_kml(output_path, 'https://drive.google.com/drive/folders/1Nc0sSJF1-gshAaj4k81v2x1kxkqtnhiA')
    print("Boundary and waypoints KML uploaded to Google Drive")

    return output_path


def main():

    final_hotspots = []
    source_markers   = []   # will hold (lat, lon, description)
    
    user_input = input(f"Press Enter to takeoff") 

    arm_and_takeoff(the_connection, 10)

    wait_for_takeoff_completion(10)

    # set_mode_loiter(the_connection)
    print("Set mode to loiter, fly around and log hotspots")

    while(True):
        user_input = input(f"Press enter to log current location as hotspot, enter 'stop' to finish")

        if user_input.lower() == "stop":
            print("Stopping hotspot logging capture.")
            break  # Exit the loop

        lat_s, lon_s, _, _, _ = retrieve_gps()

        final_hotspots.append([lat_s, lon_s])

    print("Fly to the fire source")
    input("Press Enter to mark location:")  # wait for pilot to arrive
    lat_s, lon_s, _, _, _ = retrieve_gps()
    desc = input("Enter a description for this source of fire: ")
    source_markers.append((lat_s, lon_s, desc))

    # generate and upload kml of final hotspots
    # merge any final picks that are still within, say, 2 m
    clustered_final_hotspots = cluster_hotspots(final_hotspots, threshold_m=2.0)
    # generateKML(clustered_final_hotspots, final_flags, source_markers)

    save_kml_minimal_format(clustered_final_hotspots,source_markers[0], "data/kml_source_files/hotspots_minimal.kml")

    input("Press Enter to return to launch:")

    set_mode_RTL(the_connection)



           

   


if __name__ == "__main__":

    main()