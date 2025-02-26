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
from picamzero import Camera
from src.functions.detect_hotspots import detect_hotspots

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

# Pin configuration
GPIO_PIN = 23

# Setup
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
GPIO.setup(GPIO_PIN, GPIO.IN)



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
            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
            return lat, lon, alt
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
            'latitude': lat,
            'longitude': lon,
            'altitude': altitude
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
            'latitude': lat,
            'longitude': lon,
            'altitude': altitude[0] if altitude else 0.0  # Default altitude is 0 if not provided
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
        0b0000111111111000,  # Type mask (ignore velocity, acceleration, and yaw)
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
    print(f"Waiting for GPIO 23 to change state from {initial_state}...")
    
    # Loop until the current state differs from the initial state
    while True:
        current_state = GPIO.input(GPIO_PIN)
        if current_state != initial_state:
            print(f"GPIO 23 changed state to {current_state}")
            return
        time.sleep(0.5)  # Adjust polling rate as needed

def reposition_drone_over_hotspot(connection, threshold=0.5):
    """
    Grabs location based on image,
    call reposition function untill within acceptable distance from hotspot
    """
    while True:
        # Capture image from camera
        cam = Camera()
        image = cam.capture_array()

        x_offset, y_offset, z_offset = get_offset(connection=connection, image=image)

        if x_offset is None or y_offset is None:
            print("Error retreiving offset")
            return False
        
        distance = math.sqrt(x_offset**2 + y_offset**2)
        print(f"Distance to target: {distance:.2f} meters")

        if distance < threshold:
            return True
        
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            current_x = msg.x
            current_y = msg.y
            current_z = msg.z

        target_x = current_x + x_offset
        target_y = current_y + y_offset
        target_z = current_z + z_offset
        
        # Send reposition command in body-relative frame
        send_body_offset_local_position(connection, x_offset, y_offset, z_offset)
        
        wait_for_position_target_local(connection, target_x, target_y, target_z)
        

def wait_for_position_target_local(connection, target_x, target_y, target_z, threshhold,interval=0.5, speed_threshold=0.05):
    #Do Stuff to check current local position and compare it to target
    while True:
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is None:
            print("Failed to retrieve position data.")
            return False

        updated_x = msg.x
        updated_y = msg.y
        updated_z = msg.z

        vx = msg.vx
        vy = msg.vy
        vz = msg.vz
        speed = math.sqrt(vx**2 + vy**2 + vz**2)

        x_distance = abs(updated_x - target_x)
        y_distance = abs(updated_y - target_y)
        z_distance = abs(updated_z - target_z)

        distance = math.sqrt(x_distance**2 + y_distance**2 + z_distance**2)

        if (distance < threshhold and speed < speed_threshold):
            return True
        
        time.sleep(interval)

def get_offset(connection, image, fov_x=62.2, fov_y=48.8, image_width=3280, image_height=2464):
    """
    Detects a hotspot in the image, estimates its offset from the drone
    """
    # Get altitude relative to ground (AGL)
    altitude = get_relative_altitude(connection)
    if altitude is None:
        print("Failed to retrieve relative altitude.")
        return None, None
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    # Detect hotspots
    hotspots = detect_hotspots(gray_image, threshold=0.9)
    if hotspots is None or len(hotspots) == 0:
        print("No hotspots detected.")
        return None, None
    
    # Assume the largest detected hotspot is the target
    target_hotspot = hotspots[0]  # Selecting the first hotspot detected
    print(f"Detected hotspot at: {target_hotspot}")
    
    # Convert image coordinates to movement offsets
    img_center_x = image_width / 2
    img_center_y = image_height / 2
    
    # Compute meters per pixel scale dynamically based on altitude
    fov_x_rad = math.radians(fov_x)
    fov_y_rad = math.radians(fov_y)
    meters_per_pixel_x = (2 * altitude * math.tan(fov_x_rad / 2)) / image_width
    meters_per_pixel_y = (2 * altitude * math.tan(fov_y_rad / 2)) / image_height
    
    x_offset = (target_hotspot[0] - img_center_x) * meters_per_pixel_x
    y_offset = (target_hotspot[1] - img_center_y) * meters_per_pixel_y
    z_offset = 0

    print(f"Offset to hotspot: {x_offset:.2f}m forward/backward, {y_offset:.2f}m right/left")

    return x_offset, y_offset, z_offset

def get_relative_altitude(connection):
    """
    Retrieves the drone's altitude relative to the ground (AGL) in meters.
    """
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        return msg.relative_alt / 1000.0  # Convert from mm to meters
    return None

def send_body_offset_local_position(connection, x_offset, y_offset, z_offset):
    """
    Sends a SET_POSITION_TARGET_LOCAL_NED command to reposition the vehicle.
    """
    connection.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
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

def main():

    input("Press enter")  # Wait for user to press Enter
    
    pitch = 0.0  # Replace with actual pitch value
    
    reposition_drone_over_hotspot(the_connection, pitch)
    
    print("Process completed.")
    


if __name__ == "__main__":
    main()
