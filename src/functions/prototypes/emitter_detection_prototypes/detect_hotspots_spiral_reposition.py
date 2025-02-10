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


def main():

    subdir = "data/kml_source_files"

    filename = "vantreight_buckets.kml"

    image_path = "data/field_data/ir_detection_test_images/aerial3.jpg"
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    output_kml_path = f"data/kml_source_files/hotspots_{timestamp}.kml"
    
    boundary_coords = [
        (48.4941672, -123.3105442),
        (48.4933922, -123.3064324),
        (48.4911987, -123.3076769),
        (48.4919862, -123.3122421)
    ]

    boundary_polygon = Polygon(boundary_coords)
    
    # Set mode GUIDED
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        4, 0, 0, 0, 0, 0  # Custom mode for GUIDED
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)

    time.sleep(5)

    arm_throttle(the_connection)
    
    takeoff(the_connection, 50)
    
    # Set mode LOITER
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        5, 0, 0, 0, 0, 0  # Custom mode for LOITER
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    
    wait_for_switch()
    
    # Set mode GUIDED
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        4, 0, 0, 0, 0, 0  # Custom mode for GUIDED
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    
    current_lat, current_lon, current_alt = get_latest_gps(the_connection)

    waypoints = generate_path_custom_boundary(current_lat,current_lon,20,boundary_polygon)

    coordinates_dict = format_waypoints_to_dict(waypoints)

    # Initialize hotspots list
    detected_hotspots = []
    get_gps_points = []

    for marker_name, coords in coordinates_dict.items():
        # Extract latitude, longitude, and altitude for this marker
        latitude = coords['latitude']
        longitude = coords['longitude']
        altitude = 50
        
        # Call the function to set the target position
        send_set_position_target_global_int(the_connection, latitude, longitude, altitude, coordinate_frame=6)
        
        # Now wait until the target position is reached
        wait_for_position_target(the_connection, latitude, longitude, altitude, threshold=0.5)
        
        print(f"Position target for '{marker_name}' reached.\n")

        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        hotspots = detect_hotspots_with_mask(image, threshold=0.7)
        print(f"Detected hotspots: {hotspots}")
        # Get the latest GPS coordinates from drone
        lat, lon, alt = get_latest_gps(the_connection)
        get_gps_points.append({"lat": lat, "lon": lon})

        # Map hotspots to GPS coordinates using `get_hotspots_gps`
        if hotspots.size > 0:
            distorted_points = [[x, y] for x, y in hotspots]
            dist_pts = np.array(distorted_points, dtype=np.float32)
            pitch = math.radians(-90)
            azimuth = 0.0  # Replace with actual azimuth reading
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

    set_mode_rtl(the_connection)
    
    # Define icon URLs for each group
    hotspot_icon = "http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png"
    gps_icon = "http://maps.google.com/mapfiles/kml/pushpin/grn-pushpin.png"
    waypoint_icon = "http://maps.google.com/mapfiles/kml/pushpin/blue-pushpin.png"

    hotspot_count = 1
    # Generate KML file
    kml = simplekml.Kml()
    for point in detected_hotspots:
        pnt = kml.newpoint(name=f"Hotspot {hotspot_count}", coords=[(point["lon"], point["lat"])])  # Lon, Lat
        pnt.style.iconstyle.icon.href = hotspot_icon
        hotspot_count += 1
    
    gps_point_count = 1
    for point in get_gps_points:
        pnt = kml.newpoint(name=f"Get GPS point {gps_point_count}", coords=[(point["lon"], point["lat"])])  # Lon, Lat
        pnt.style.iconstyle.icon.href = gps_icon
        gps_point_count += 1

    waypoint_count = 1
    for marker_name, coords in coordinates_dict.items():
        pnt = kml.newpoint(name=f"Get GPS point {waypoint_count}", coords=[(coords['longitude'], coords['latitude'])])  # Lon, Lat
        pnt.style.iconstyle.icon.href = waypoint_icon
        waypoint_count += 1

    
    # Save KML file
    os.makedirs(os.path.dirname(output_kml_path), exist_ok=True)
    kml.save(output_kml_path)
    upload_kml(output_kml_path, 'https://drive.google.com/drive/folders/1GW56sU4zJuf8If8B6NgVultpZ_C2QT2V')
    print(f"KML file saved to {output_kml_path}")


if __name__ == "__main__":
    main()
