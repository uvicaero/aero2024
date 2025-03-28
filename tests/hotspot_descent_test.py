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

import time
import datetime

latest_gps = None  # Stores latest GPS data (lat, lon, alt)
latest_attitude = None  # Stores latest attitude data (pitch, roll, yaw)
latest_local = None  # Stores latest LOCAL_POSITION_NED data (x, y, z)

latest_local = None  # Now stores (x, y, z, vx, vy, vz)

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
                    latest_gps = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3, msg.vx / 100, msg.vy / 100, msg.vz / 100)

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

    lat, lon, rel_alt, vx, vy, vz = latest_gps if latest_gps else (None, None, None, None, None, None)

    # Extract only pitch and yaw (ignore roll)
    if latest_attitude:
        pitch, _, yaw = latest_attitude  # Ignore roll using `_`
    else:
        pitch, yaw = None, None  # Default values if attitude is missing

    return lat, lon, rel_alt, vx, vy, vz, pitch, yaw

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

def get_latest_gps(connection):
    """
    Fetches the latest GPS message from the MAVLink connection.
    """
    try:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            yaw = msg.hdg / 100.0 if msg.hdg != 65535 else None  # Convert centidegrees to degrees, handle invalid value
            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters, Yaw: {yaw} degrees")
            return lat, lon, alt, yaw
    except Exception as e:
        print(f"Error fetching GPS: {e}")
        return None, None, None
    


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

def send_set_position_target_global_int(connection, latitude, longitude, altitude, vel = 2, coordinate_frame=11):
    """
    Sends a SET_POSITION_TARGET_GLOBAL_INT command to reposition the vehicle.

    Args:
        connection (mavutil.mavlink_connection): MAVLink connection object.
        latitude (float): Target latitude in degrees.
        longitude (float): Target longitude in degrees.
        altitude (float): Target altitude in meters.
        coordinate_frame (int): MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (default: 6).
    """
    _, _, _, _, _, _, _, yaw = retrieve_gps()

    connection.mav.set_position_target_global_int_send(
        connection.target_system,  # Target system
        connection.target_component,  # Target component
        0,  # Time boot ms (not used)
        coordinate_frame,  # Coordinate frame (relative altitude)
        0b000111111000,  # Type mask 
        int(latitude * 1e7),  # Latitude in 1E7 degrees
        int(longitude * 1e7),  # Longitude in 1E7 degrees
        altitude,  # Altitude in meters
        vel, vel, 1.5,  # Velocity (not used)
        0, 0, 0,  # Acceleration (not used)
        yaw, math.radians(20)  # Yaw and yaw rate (not used)
    )
    print(f"Set position target global int sent to ({latitude}, {longitude}, {altitude})")

def issue_altitude_change_agl(connection, alt_agl, rate_of_climb=1.0):
    """
    Commands the drone to change altitude above ground level (AGL) without specifying lat/lon.

    Parameters:
        alt_agl (float): Desired altitude above ground level in meters.
        rate_of_climb (float): Rate of climb/descent in m/s.
    """
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
        0,                      # Confirmation
        alt_agl,                # Target altitude (AGL) in meters
        mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,  # Altitude frame (terrain-relative)
        rate_of_climb,          # Climb/descent rate (m/s)
        0, 0, 0, 0              # Unused parameters
    )

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

def reposition_drone_over_hotspot(connection, camera, threshold=0.5, k_p=0.8):
    """
    Grabs the bucket location via bucket detection and repositions the drone until
    it is within an acceptable distance from the bucket.
    """
    while True:
        # Use the new get_offset (which calls detectBucket) with a 5-second video
        x_offset, y_offset, z_offset = get_offset(connection, camera, videoLength=0.5)
        if x_offset is None or y_offset is None:
            print("Error retrieving offset")
            return False
        
        distance = math.sqrt(x_offset**2 + y_offset**2)
        print(f"Distance to target: {distance:.2f} m")
        if distance < threshold:
            return True
        
        current_x, current_y, current_z, _, _, _, yaw = retrieve_local()
        if current_x is None or current_y is None or current_z is None:
            print("Error retrieving local position.")
            return False

        move_x = k_p * x_offset
        move_y = k_p * y_offset
        move_z = k_p * z_offset
        yaw_rad = float(yaw)
        target_x = current_x + (move_x * math.cos(yaw_rad) - move_y * math.sin(yaw_rad))
        target_y = current_y + (move_x * math.sin(yaw_rad) + move_y * math.cos(yaw_rad))
        target_z = current_z + z_offset
        
        send_body_offset_local_position(connection, move_x, move_y, move_z)
        wait_for_position_target_local(connection, target_x, target_y, target_z)

def wait_for_position_target_local(connection, target_x, target_y, target_z, threshold=0.5, interval=0.5, speed_threshold=0.05):
    while True:
        updated_x, updated_y, updated_z, vx, vy, vz, _ = retrieve_local()
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        x_distance = abs(updated_x - target_x)
        y_distance = abs(updated_y - target_y)
        z_distance = abs(updated_z - target_z)
        distance = math.sqrt(x_distance**2 + y_distance**2 + z_distance**2)
        print(f"[DEBUG] Distance to target: {distance:.2f} m, Speed: {speed:.2f} m/s")
        if distance < threshold and speed < speed_threshold:
            print("[SUCCESS] Target position reached.")
            return True
        time.sleep(interval)

def get_offset(connection, camera, videoLength=1, fov_x=102, fov_y=66, image_width=1280 , image_height=720):
    """
    Uses bucket detection to determine the target’s pixel center and calculates
    the corresponding movement offsets based on the camera’s field of view and altitude.
    """
    lat, lon, rel_alt, vx, vy, vz, pitch, azimuth = retrieve_gps()
    if rel_alt is None:
        print("Failed to retrieve relative altitude.")
        return None, None, 0
    center = detectBucket(videoLength, camera)
    if center == (None, None):
        print("No circles detected.")
        return None, None, 0
    target_hotspot = center
    print(f"Detected averaged center: {target_hotspot}")
    img_center_x = image_width / 2
    img_center_y = image_height / 2
    fov_x_rad = math.radians(fov_x)
    fov_y_rad = math.radians(fov_y)
    meters_per_pixel_x = (2 * rel_alt * math.tan(fov_x_rad / 2)) / image_width
    meters_per_pixel_y = (2 * rel_alt * math.tan(fov_y_rad / 2)) / image_height
    x_offset = -(target_hotspot[1] - img_center_y) * meters_per_pixel_y
    y_offset = (target_hotspot[0] - img_center_x) * meters_per_pixel_x
    z_offset = 0
    print(f"Offset to bucket: {x_offset:.2f} m forward/backward, {y_offset:.2f} m right/left")
    return x_offset, y_offset, z_offset

def send_body_offset_local_position(connection, x_offset, y_offset, z_offset):
    time_boot_ms = int(round(time.time() * 1000)) & 0xFFFFFFFF
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

def detectBucket(videoLength, camera):
    print("Detection Started")
    start = time.time()
    timePassed = 0

    listOfCenters = []


    # Set up Matplotlib interactive mode
    #plt.ion()
    #fig, ax = plt.subplots()
    #im_display = ax.imshow(np.zeros((480, 640), dtype=np.uint8), cmap="gray")  # Empty image placeholder
    #plt.axis("off")  # Hide axes

    while timePassed < videoLength:
        # Capture grayscale image directly from PiCamera
        gray = cv2.cvtColor(camera.capture_array("main"), cv2.COLOR_BGR2GRAY)

        # Apply median blur
        gray = cv2.medianBlur(gray, 5)

        # Detect circles using HoughCircles
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT_ALT, dp=1.5, minDist=rows / 8,
                                  param1=300, param2=0.92,
                                  minRadius=0, maxRadius=0)

        # Convert grayscale to 3-channel image for drawing circles
        src_display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                print(f"Detected circle: Center={i[0], i[1]}, Radius={i[2]}")
                circle_info = (i[0], i[1], i[2])
                
                # circle center
                center = (i[0], i[1])
                cv2.circle(src_display, center, 1, (0, 100, 100), 3)
                listOfCenters.append(circle_info)
                # circle outline
                radius = i[2]
                cv2.circle(src_display, center, radius, (255, 0, 255), 3)

        # Update Matplotlib display
        #im_display.set_data(src_display)
        #plt.pause(0.01)  # Allow time for the image to refresh

        # Update time
        timePassed = time.time() - start
    #plt.ioff()  # Turn off interactive mode
    #plt.show()  # Show final frame

    return averageCenters(listOfCenters)

# Params: centers - List of all the circles detected and their info [x, y, rad]
# Return: (x_avg, y_avg) as a Tuple
def averageCenters(centers):
    if len(centers) < 1:
        return (-1, -1, -1) # error return value
    
    radius_values = []
    for c in centers:
        radius_values.append(c[2])

    rad_max = max(radius_values)
    rad_max_thresh = rad_max*0.9 # 90% of the max radius

    x_values = []
    y_values = []
    radius_values = []
    for c in centers:
        # Not factor in any circles less than the biggest rad
        if c[2] > rad_max_thresh:
            x_values.append(c[0])
            y_values.append(c[1])
            radius_values.append(c[2])
    print("Mean: ", rad_max)
    print("Mean Thresh: ", rad_max_thresh)

    # Possible idea to calculate average again and 
    # not factor in any values outside of the std dev

    #return (int(np.mean(x_values)), int(np.mean(y_values)), int(np.mean(radius_values))) # Return with radius
    return (int(np.mean(x_values)), int(np.mean(y_values)))

def distance_between_gps(lat1, lon1, lat2, lon2):
    """Calculate approximate distance in meters between two GPS coordinates."""
    R = 6371000  # radius of Earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

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
        current_lat, current_lon, current_alt, vx, vy, vz, _, _ = retrieve_gps()

        horizontal_distance = distance_between_gps(current_lat, current_lon, target_lat, target_lon)
        vertical_distance = abs(current_alt - target_alt)
        velocity = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        if horizontal_distance <= tolerance_m and vertical_distance <= alt_tolerance and velocity <= velocity_threshold:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start >= stable_time:
                print("Drone has stabilized at the target coordinates.")
                break
        else:
            stable_start = None  # Reset stable time if it drifts or moves

        time.sleep(0.5)  # Polling interval

def wait_until_altitude(target_alt, alt_tolerance=0.5, velocity_threshold=0.2, stable_time=1.0):
    """
    Waits until the drone reaches and stabilizes at a target altitude.

    Parameters:
        target_alt (float): Target altitude
        alt_tolerance (float): Vertical altitude tolerance in meters
        velocity_threshold (float): Vertical velocity threshold (m/s) to determine stability
        stable_time (float): Duration (seconds) drone must be stable at altitude
    """

    stable_start = None

    while True:
        _, _, current_alt, _, _, vz, _, _ = retrieve_gps()

        vertical_distance = abs(current_alt - target_alt)
        vertical_velocity = abs(vz)

        if vertical_distance <= alt_tolerance and vertical_velocity <= velocity_threshold:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start >= stable_time:
                print("Drone has stabilized at the target altitude.")
                break
        else:
            stable_start = None  # Reset stable time if altitude drifts or moves

        time.sleep(0.5)  # Polling interval

def imageToHotspotCoordinates(image):
    """
    Gets list of hotspot lat/lon from an image

    Parameters:
        image: either static image from a file or taken live from picam2
    Return:
        detected_hotspots: a list of every hotspot detected in the image as lat/lon pairs
        get_gps_points: a lat/lon object of where the drone was when the photo was taken
    """
    detected_hotspots = []

    hotspots = detect_hotspots_with_mask(image, threshold=0.7)
    print(f"Detected hotspots: {hotspots}")
    # Get the latest GPS coordinates from drone
    lat, lon, alt, yaw = get_latest_gps(the_connection) ############################SHOULD we use get_latest_gps, retrieve_gps or retrieve_local
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
                detected_hotspots.append({"lat": gps_point[0], "lon": gps_point[1]})
        else:
            print(f"Error: Invalid GPS hotspots output {gps_hotspots}")

    return detected_hotspots


def merge_hotspots(hotspot_positions, merge_distance=2):
    """
    Merges clusters of nearby hotspot positions by replacing them with their average center.
    
    Parameters:
        hotspot_positions list of (object {lat: x, lon: y}): List of objects for detected hotspots.
        merge_distance (float): Maximum distance in meters to consider two points as the same hotspot.
    
    Returns:
        list of objects: Unique (object {lat: x, lon: y}) hotspot positions after merging.
    """
    if not hotspot_positions:
        return []

    # Approximate conversion factors (meters per degree at given latitude)
    lat_factor = 111230.9531021928  # Meters per degree of latitude
    lon_factor = 71546.90282746412  # Meters per degree of longitude (approximate at this latitude)

    unique_hotspots = []  # List of merged hotspots
    unprocessed = set(range(len(hotspot_positions)))  # Keep track of unprocessed hotspots

    hotspot_positions = np.array(hotspot_positions)

    while unprocessed:
        index = unprocessed.pop()  # Take an unprocessed hotspot
        lat, lon = hotspot_positions[index].lat, hotspot_positions[index].lon

        # Find all nearby hotspots within the merge distance
        nearby_indices = []
        cluster_lat, cluster_lon = [lat], [lon]

        for i in list(unprocessed):
            u_lat, u_lon = hotspot_positions[i]

            # Convert degrees to meters and compute distance
            lat_dist = (lat - u_lat) * lat_factor
            lon_dist = (lon - u_lon) * lon_factor
            distance = np.sqrt(lat_dist**2 + lon_dist**2)  # Euclidean distance in meters

            if distance < merge_distance:
                nearby_indices.append(i)
                cluster_lat.append(u_lat)
                cluster_lon.append(u_lon)

        # Remove processed indices from unprocessed set
        for i in nearby_indices:
            unprocessed.remove(i)

        # Compute the average position of the cluster
        avg_lat = np.mean(cluster_lat)
        avg_lon = np.mean(cluster_lon)
        unique_hotspots.append({"lat": avg_lat, "lon": avg_lon})

    return unique_hotspots
    # REMEMBER TO SORT list in a way that makes sense to fly in

def generateKML(hotspots):

    # File Save Location
    output_kml_path = "data/kml_source_files/hotspots.kml"

    # Generate KML file

    kml = simplekml.Kml()
    hotspot_count = 1
    for point in hotspots:
        kml.newpoint(name=f"Hotspot {hotspot_count}", coords=[(point["lon"], point["lat"])])  # Lon, Lat
        hotspot_count += 1
    
    """
    gps_point_count = 1
    for point in get_gps_points:
        kml.newpoint(name=f"Get GPS point {gps_point_count}", coords=[(point["lon"], point["lat"])])  # Lon, Lat
        gps_point_count += 1
    """

    # Save KML file
    os.makedirs(os.path.dirname(output_kml_path), exist_ok=True)
    kml.save(output_kml_path)
    print(f"KML file saved to {output_kml_path}")
        

def main():

    # Images for testing
    image_path_80m = "data/field_data/jan_8th_test_photos_run_1/photo_alt_80m.jpg"
    image_path_50m = "data/field_data/jan_8th_test_photos_run_1/photo_alt_50m.jpg"
    image_path_20m = "data/field_data/jan_8th_test_photos_run_1/photo_alt_20m.jpg"

    # Initialize hotspots list

    detected_hotspots_80m = []
    avg_hotspot_clusters_80m = []

    detected_hotspots_50m = []
    detected_hotspots_20m = []


    time.sleep(2)
    # 1. Go to the specified coordinates at 80m and take a photo
    send_set_position_target_global_int(the_connection, 48.49276, -123.30896, 10, 11)
    wait_until_reached(the_connection, 48.49276, -123.30896, 10)

    print(f"Read image...")
    # 2. Find any collections of hotspots in the photo and group each collection into a single average point
    image = cv2.imread(image_path_80m, cv2.IMREAD_GRAYSCALE)

    # Scans a photo and returns list of lat/long
    detected_hotspots_80m = imageToHotspotCoordinates(image)
    print(f"All Hotspots at 80m: {detected_hotspots_80m}")

    # Take list and find averages for clusters and add to list
    avg_hotspot_clusters_80m = merge_hotspots(detected_hotspots_80m)
    print(f"Merged Hotspots at 80m: {detected_hotspots_80m}")

    # 3. Descend to 50m at each of the average points collected above
    for point in avg_hotspot_clusters_80m:
        send_set_position_target_global_int(the_connection, point.lat, point.lon, 50, 11)
        wait_until_reached(the_connection, point.lat, point.lon, 50)

        # Take photo (or use test photo)
        image = cv2.imread(image_path_50m, cv2.IMREAD_GRAYSCALE)
        hotspot = imageToHotspotCoordinates(image)
        empty_photos = 1

        # If no hotspot found, fly up 5m and take another photo
        while hotspot.len() == 0:
            issue_altitude_change_agl(the_connection, (50+(5*empty_photos)), 1) #################### IS THIS 5m relative up or go to 5m off terrain?
            wait_until_altitude(50+(5*empty_photos))
            # Take another photo (or use test photo)
            image = cv2.imread(image_path_80m, cv2.IMREAD_GRAYSCALE)
            hotspot = imageToHotspotCoordinates(image)
            empty_photos += 1

        # Once hotspot has been detected, add hotspot to list
        detected_hotspots_50m.extend(hotspot)

    print(f"Hotspots at 50m: {detected_hotspots_50m}")



    # 4. Descend to 20m at each new point and add the final coordinate of each hotspot to the kml file
    for point in detected_hotspots_50m:
        send_set_position_target_global_int(the_connection, point.lat, point.lon, 20, 11)
        wait_until_reached(the_connection, point.lat, point.lon, 20)

        # Take photo (or use test photo)
        image = cv2.imread(image_path_20m, cv2.IMREAD_GRAYSCALE)
        hotspot = imageToHotspotCoordinates(image)
        empty_photos = 1

        # If no hotspot found, fly up 5m and take another photo
        while hotspot.len() == 0:
            issue_altitude_change_agl(the_connection, (20+(5*empty_photos)), 1) #################### IS THIS 5m relative up or go to 5m off terrain?
            wait_until_altitude(20+(5*empty_photos))
            # Take another photo (or use test photo)
            image = cv2.imread(image_path_80m, cv2.IMREAD_GRAYSCALE)
            hotspot = imageToHotspotCoordinates(image)
            empty_photos += 1

        # Once hotspot has been detected, add hotspot to list
        detected_hotspots_20m.extend(hotspot)

    print(f"Hotspots at 20m: {detected_hotspots_20m}")


    generateKML(detected_hotspots_20m)


if __name__ == "__main__":
    main()
