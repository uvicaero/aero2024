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

latest_gps = None

latest_attitude = None  # Stores latest attitude data (pitch, roll, yaw)

def mavlink_listener(the_connection):
    """
    Single background thread that alternates between listening for GPS and Attitude messages.
    Updates `latest_gps` and `latest_attitude` whenever new data is received.
    """
    global latest_gps, latest_attitude

    #print("[DEBUG] MAVLink Listener thread started.")

    gps_toggle = True  # Toggle flag to alternate between GPS and Attitude

    while True:
        try:
            if not the_connection or not the_connection.port:
                #print("[ERROR] MAVLink connection lost. Retrying in 2 seconds...")
                time.sleep(2)
                continue  # Skip iteration until connection is restored

            # Alternate between fetching GPS and Attitude
            if gps_toggle:
                msg_type = "GLOBAL_POSITION_INT"
            else:
                msg_type = "ATTITUDE"

            msg = the_connection.recv_match(type=msg_type, blocking=True, timeout=5)

            if msg:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                if msg_type == "GLOBAL_POSITION_INT":
                    latest_gps = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3)
                    #print(f"[DEBUG] {timestamp} - GPS Update: Lat={latest_gps[0]}, Lon={latest_gps[1]}, Alt={latest_gps[2]}m")

                elif msg_type == "ATTITUDE":
                    latest_attitude = (msg.pitch, msg.roll, msg.yaw)
                    #print(f"[DEBUG] {timestamp} - Attitude Update: Pitch={latest_attitude[0]} rad, Roll={latest_attitude[1]} rad, Azimuth={latest_attitude[2]} rad")

                gps_toggle = not gps_toggle  # Flip flag to fetch the other message type next time

            else:
                print(f"[WARNING] No {msg_type} data received within timeout (5s). Retrying...")

        except Exception as e:
            print(f"[ERROR] Exception in MAVLink Listener: {e}")
            time.sleep(2)  # Prevent excessive logging



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

def get_latest_gps(the_connection):
    """
    Fetches the latest GPS message from the MAVLink connection.
    """
    try:
        # Flush buffer to remove old messages
        while True:
            old_msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if old_msg is None:
                break  # Exit loop when buffer is empty
        # Now, wait for a fresh GPS message
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            #print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
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

def main():
    # Timestamp for KML output
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_kml_path = f"data/kml_source_files/manual_hotspot_test_{timestamp}.kml"
    
    subdir = "data/kml_source_files"

    detected_hotspots = {}
    capture_spots = []
    true_hotspot_points = extract_coordinates_to_dict(subdir, filename)

    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (3280, 2464)}  # Maximum resolution
    )
    picam2.configure(config)
    picam2.start()

    print("\nReady to capture images for hotspot detection.")
    print("Press Enter to capture each image.\n")
    i = 0  # Counter for images
    while True:
        user_input = input(f"Press Enter to capture image {i + 1} (or type 'stop' to finish)...")
        
        if user_input.lower() == "stop":
            print("Stopping image capture.")
            break  # Exit the loop

        print("\nCapturing image...")
        rgb_image = picam2.capture_array("main")
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

        # Get current GPS location, altitude, pitch, and azimuth
        lat, lon, rel_alt, pitch, azimuth = retrieve_gps()
        capture_spots.append({"lat": lat, "lon": lon})
        print(f"GPS: lat={lat}, lon={lon}, rel_alt={rel_alt}m, pitch={pitch} rad, azimuth={azimuth} rad")

        print("Running hotspot detection...")
        hotspots = detect_hotspots(gray_image, threshold=0.9)

        # Display the image with detected hotspots
        plt.figure(figsize=(6, 5))
        plt.imshow(gray_image, cmap="gray")
        plt.axis("off")

        if hotspots.size > 0:
            plt.scatter(hotspots[:, 0], hotspots[:, 1], color='red', marker='x', label="Detected Hotspots")

        plt.legend()
        plt.title(f"Hotspots Detected in Image {i + 1}")
        plt.show()  # Show the preview

        # Map hotspots to GPS coordinates using `get_hotspots_gps`
        if hotspots.size > 0:
            distorted_points = np.array([[x, y] for x, y in hotspots], dtype=np.float32)
            camera_pitch = pitch - math.radians(90)
            gps_hotspots = get_hotspots_gps(distorted_points, lon, lat, rel_alt, camera_pitch, azimuth)

            # Ensure output shape is (N,2)
            gps_hotspots = gps_hotspots.reshape(-1, 2) if gps_hotspots.ndim > 2 else gps_hotspots

            # Validate before appending
            if gps_hotspots.size > 0 and gps_hotspots.shape[1] == 2:
                for j, gps_point in enumerate(gps_hotspots):
                    hotspot_name = f"Hotspot_{i+1}_{j+1}"
                    detected_hotspots[hotspot_name] = {"lat": gps_point[0], "lon": gps_point[1]}
            else:
                print(f"[ERROR] Invalid GPS hotspots output: {gps_hotspots.shape} - Data: {gps_hotspots}")

        print(f"Image {i + 1} processed.\n")
        i += 1  # Increment image count

    picam2.stop()
    
    print(f"[DEBUG] Detected Hotspots: {detected_hotspots}")

    # Define icon URLs for visualization
    hotspot_icon = "http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png"
    gps_icon = "http://maps.google.com/mapfiles/kml/pushpin/blu-pushpin.png"

    # Generate KML file
    kml = simplekml.Kml()
    
    # Plot capture points
    for i, point in enumerate(capture_spots, start=1):
        pnt = kml.newpoint(name=f"Capture point {i}", coords=[(point["lon"], point["lat"], 0)])  # Lon, Lat, Alt
        pnt.style.iconstyle.icon.href = gps_icon
    
    detected_hotspot_count = 1
    # Plot detected hotspots
    for marker_name, coords in detected_hotspots.items():
        pnt = kml.newpoint(name=f"Detected Hotspot {detected_hotspot_count}", coords=[(coords["lon"], coords["lat"], 0)])  # Lon, Lat, Alt
        pnt.style.iconstyle.icon.href = hotspot_icon
        detected_hotspot_count += 1

    # Save KML file
    os.makedirs(os.path.dirname(output_kml_path), exist_ok=True)
    kml.save(output_kml_path)

    # Upload KML file to Google Drive
    upload_kml(output_kml_path, 'https://drive.google.com/drive/folders/1GW56sU4zJuf8If8B6NgVultpZ_C2QT2V')
    print(f"KML file saved to {output_kml_path}")

if __name__ == "__main__":
    main()
