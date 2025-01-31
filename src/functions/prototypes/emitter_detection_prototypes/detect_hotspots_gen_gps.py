import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import threading
import subprocess
from shapely.geometry import Point, Polygon, LineString
import os
import xml.etree.ElementTree as ET
from src.functions.prototypes.emitter_detection_prototypes.detect_hotspots_with_mask import detect_hotspots_with_mask
from src.functions.get_hotspots_gps import get_hotspots_gps

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


def load_waypoints_via_mavlink(waypoints):
    """
    Load waypoints directly via MAVLink without waiting for individual ACKs.
    :param waypoints: List of waypoints (each as a dictionary with lat, lon, alt, command).
    """
    print(f"Uploading {len(waypoints)} waypoints to the autopilot...")
    # Request home location
    request_message_command = dialect.MAVLink_command_long_message(
        target_system=the_connection.target_system,        # Target system (vehicle)
        target_component=the_connection.target_component, # Target component (autopilot)
        command=dialect.MAV_CMD_REQUEST_MESSAGE,   # Command to request a specific message
        confirmation=0,                            # Confirmation (set to 0 for no confirmation)
        param1=242,                                # Message ID to request (e.g., 242 for a specific message)
        param2=0,                                  # Reserved (set to 0 if not used)
        param3=0,                                  # Reserved (set to 0 if not used)
        param4=0,                                  # Reserved (set to 0 if not used)
        param5=0,                                  # Reserved (set to 0 if not used)
        param6=0,                                  # Reserved (set to 0 if not used)
        param7=0                                   # Reserved (set to 0 if not used)
    )

    # Send the command to the vehicle
    the_connection.mav.send(request_message_command)
    # Wait for home location to send
    home_position_message = the_connection.recv_match(type='HOME_POSITION', blocking=True).to_dict()
    home_lat = home_position_message['latitude'] * 1e-7  # Convert from int32 to float degrees
    home_lon = home_position_message['longitude'] * 1e-7
    home_alt = home_position_message['altitude'] * 1e-3  # Convert from mm to meters

    # create mission count message
    message = dialect.MAVLink_mission_count_message(
        target_system=the_connection.target_system,
        target_component=the_connection.target_component,
        count=len(waypoints) + 1,
        mission_type=dialect.MAV_MISSION_TYPE_MISSION)
    # send mission count message
    the_connection.mav.send(message)

    while True:

        # catch a message
        message = the_connection.recv_match(blocking=True)

        # convert this message to dictionary
        message = message.to_dict()

        # check this message is MISSION_REQUEST
        if message["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:

            # check this request is for mission items
            if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

                # get the sequence number of requested mission item
                seq = message["seq"]
                print(f"seq: {seq}, waypoint_index: {seq - 2}")
                # create mission item int message
                if seq == 0:
                    # create mission item int message that contains the home location (0th mission item)
                    message = dialect.MAVLink_mission_item_int_message(
                        target_system=the_connection.target_system,
                        target_component=the_connection.target_component,
                        seq=seq,
                        frame=dialect.MAV_FRAME_GLOBAL,
                        command=dialect.MAV_CMD_NAV_WAYPOINT,
                        current=0,
                        autocontinue=0,
                        param1=0,
                        param2=0,
                        param3=0,
                        param4=0,
                        x=int(home_lat * 1e7),
                        y=int(home_lon * 1e7),
                        z=home_alt,
                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)
                elif seq == 1:
                    # Add takeoff command to 20m
                    message = dialect.MAVLink_mission_item_int_message(
                        target_system=the_connection.target_system,
                        target_component=the_connection.target_component,
                        seq=seq,
                        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        command=dialect.MAV_CMD_NAV_TAKEOFF,  # Command to take off
                        current=0,
                        autocontinue=1,  # Auto-continue to the next waypoint
                        param1=0,       # Minimum pitch (not used, set to 0)
                        param2=0,       # Empty
                        param3=0,       # Empty
                        param4=0,       # Yaw angle (0 for default)
                        x=int(home_lat * 1e7),
                        y=int(home_lon * 1e7),
                        z=20)           # Takeoff altitude (20m)
                else:
                    # create mission item int message that contains a target location
                    message = dialect.MAVLink_mission_item_int_message(
                        target_system=the_connection.target_system,
                        target_component=the_connection.target_component,
                        seq=seq,
                        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        command=dialect.MAV_CMD_NAV_WAYPOINT,
                        current=0,
                        autocontinue=waypoints[seq - 2]['autocontinue'],
                        param1=0,
                        param2=0,
                        param3=0,
                        param4=0,
                        x=int(waypoints[seq - 2]['lat'] * 1e7),
                        y=int(waypoints[seq - 2]['lon'] * 1e7),
                        z=waypoints[seq - 2]['alt'],
                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)

                # send the mission item int message to the vehicle
                the_connection.mav.send(message)

        # check this message is MISSION_ACK
        elif message["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:

            # check this acknowledgement is for mission and it is accepted
            if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION and \
                    message["type"] == dialect.MAV_MISSION_ACCEPTED:
                # break the loop since the upload is successful
                print("Mission upload is successful")
                break

    print("All waypoints sent. Waiting for MISSION_ACK...")

    return True


def wait_for_waypoint(the_connection, desired_waypoint, timeout=360):
    """
    Waits until the drone reaches the specified waypoint.

    Args:
        the_connection: MAVLink connection object.
        desired_waypoint: The waypoint number to wait for.
        timeout: Maximum time to wait in seconds (default: 120).

    Returns:
        True if the waypoint is reached within the timeout, False otherwise.
    """
    print(f"Waiting for waypoint {desired_waypoint}...")
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        # Receive the MISSION_CURRENT message
        msg = the_connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
        
        if msg:
            current_waypoint = msg.seq
            print(f"Current waypoint: {current_waypoint}")
            
            # Check if the desired waypoint is reached
            if current_waypoint == desired_waypoint:
                print(f"Reached waypoint {desired_waypoint}.")
                return True
        else:
            print("Timeout waiting for MISSION_CURRENT message.")
    
    print(f"Failed to reach waypoint {desired_waypoint} within {timeout} seconds.")
    return False

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

def start_mission():
    """
    Sends the MAV_CMD_MISSION_START command to start executing the mission.
    """
    print("Starting mission...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,  # Command to start the mission
        0,  # Confirmation
        0,  # First mission item to execute (0 = start from the beginning)
        0,  # Last mission item to execute (0 = all items)
        0, 0, 0, 0, 0  # Unused parameters
    )
    return wait_for_ack(mavutil.mavlink.MAV_CMD_MISSION_START)

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

def create_mission_waypoints(coordinates_dict):
    """
    Create waypoints for the mission, generating only one waypoint per bucket with autocontinue set to 0.
    
    Args:
        coordinates_dict (dict): Dictionary with names as keys and lat/lon/alt as values.
                                 Example: {'Reservoir': {'latitude': 48.4928683, 'longitude': -123.3092024, 'altitude': 0.0}}
    
    Returns:
        tuple: (waypoints, autohold_indices)
               waypoints: List of dictionaries with fields (lat, lon, alt, autocontinue).
               autohold_indices: List of waypoint indices where autocontinue = 0.
    """
    # Extract reservoir and buckets from the dictionary
    reservoir = coordinates_dict['Reservoir']
    buckets = {k: v for k, v in coordinates_dict.items() if k.startswith('Bucket')}
    
    # Sort buckets by name (Bucket 1, Bucket 2, etc.)
    sorted_buckets = sorted(buckets.items(), key=lambda x: x[0])

    # Initialize waypoints list and autohold index list
    waypoints = []
    autohold_indices = []
    
    # Define reservoir coordinates
    reservoir_lat, reservoir_lon = reservoir['latitude'], reservoir['longitude']
    
    # Loop through each bucket and add a single waypoint with autocontinue = 0
    for bucket_name, bucket_coords in sorted_buckets:
        bucket_lat, bucket_lon = bucket_coords['latitude'], bucket_coords['longitude']
        
        # Single waypoint for the current bucket
        bucket_waypoint = {'lat': bucket_lat, 'lon': bucket_lon, 'alt': 5, 'autocontinue': 0}
        waypoints.append(bucket_waypoint)
        autohold_indices.append(len(waypoints) - 1)
    
    return waypoints, autohold_indices

def wait_for_autohold(master, autohold_indices):
    """
    Wait for the drone to reach a waypoint in autohold_indices and pause.
    
    Args:
        master: MAVLink connection object.
        autohold_indices: List of waypoint indices where autocontinue = 0.
    """
    print("Waiting for the drone to reach an autohold waypoint...")
    while True:
        # Listen for MISSION_CURRENT messages
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
        if not msg:
            continue
        
        # Extract the current waypoint index
        current_wp = msg.seq
        print(f"Current waypoint: {current_wp}")
        
        # Check if the current waypoint is in the autohold list
        if current_wp-3 in autohold_indices:
            print(f"Drone has reached autohold waypoint {current_wp} (autocontinue = 0).")
            return
            
def is_mission_completed(master, total_waypoints):
    """
    Check if the mission has been completed by monitoring the current waypoint.
    
    Args:
        master: MAVLink connection object.
        total_waypoints: Total number of waypoints in the mission.
    
    Returns:
        bool: True if the mission is completed, False otherwise.
    """
    # Listen for the current waypoint message
    msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
    if not msg:
        return False

    current_wp = msg.seq
    print(f"Current waypoint: {current_wp}")
    
    # Check if the current waypoint is the last waypoint
    if current_wp == total_waypoints - 1:
        print("Mission completed.")
        return True

    return False

def continue_mission_from_current(master):
    """
    Continue the mission starting from the current waypoint as reported by the autopilot.

    Args:
        master: MAVLink connection object.
    """
    print("Checking the current waypoint...")
    
    # Wait for the MISSION_CURRENT message to get the current waypoint
    while True:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
        if msg:
            current_wp = msg.seq
            print(f"Current waypoint reported by autopilot: {current_wp}")
            break

    # Send the MISSION_SET_CURRENT command to resume from the current waypoint
    master.mav.mission_set_current_send(
        master.target_system,   # Target system (e.g., drone)
        master.target_component,  # Target component (e.g., autopilot)
        current_wp+1  # Index of the current waypoint
    )
    print(f"Mission resumed at waypoint {current_wp+1}.")

def set_servo_pwm(master, servo_channel, pwm_value):
    """
    Set a servo to a specific PWM value using MAVLink.

    Args:
        master: MAVLink connection object.
        servo_channel: The channel of the servo to control (e.g., 1 for SERVO1).
        pwm_value: The desired PWM value (typically between 1000 and 2000 microseconds).
    """
    print(f"Setting servo {servo_channel} to PWM value {pwm_value}...")

    # Send the MAVLink command to set the servo
    master.mav.command_long_send(
        master.target_system,            # Target system (e.g., drone)
        master.target_component,         # Target component (e.g., autopilot)
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command to set a servo
        0,                               # Confirmation (set to 0)
        servo_channel,                   # Servo output channel (1-based index)
        pwm_value,                       # PWM value to set (in microseconds)
        0, 0, 0, 0, 0                   # Unused parameters
    )
    print(f"Command sent to set servo {servo_channel} to {pwm_value}.")

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

def pause_mission(master):
    """
    Pause the mission by switching to GUIDED mode.
    
    Args:
        master: MAVLink connection object.
    """
    print("Pausing mission...")
    # Set mode GUIDED
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        4, 0, 0, 0, 0, 0  # Custom mode for GUIDED
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    print("Mission paused. Drone is now in GUIDED mode.")

def resume_mission(master):
    """
    Resume the mission from a specific waypoint index.
    
    Args:
        master: MAVLink connection object.
        waypoint_index: Index of the waypoint to resume from.
    """
    print("Resuming mission...")
    # Switch back to AUTO mode
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        3, 0, 0, 0, 0, 0  # Custom mode for AUTO
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)

    # Set the current mission waypoint
    # master.mav.mission_set_current_send(
    #     master.target_system,
    #     master.target_component,
    #     waypoint_index  # Resume from this waypoint
    # )
    # print(f"Mission resumed at waypoint {waypoint_index}.")


def main():

    subdir = "data/kml_source_files"

    filename = "vantreight_buckets.kml"

    image_path = "data/field_data/ir_detection_test_images/photo_alt_20m_run_1.jpg"

    output_kml_path = "data/kml_source_files/hotspots.kml"

    coordinates_dict = extract_coordinates_to_dict(subdir, filename)

    waypoints,autohold_indices = create_mission_waypoints(coordinates_dict)

    # Initialize hotspots list
    detected_hotspots = []

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

    # Load waypoints
    if not load_waypoints_via_mavlink(waypoints):
        print("Failed to upload waypoints. Exiting.")
        return

    time.sleep(5)

    arm_throttle(the_connection)
    
    # Start the mission
    if not start_mission():
        print("Failed to start the mission. Exiting.")
        return

    time.sleep(5)

    while True:
        if is_mission_completed(the_connection, len(waypoints)+1):
            break
        wait_for_autohold(the_connection, autohold_indices)
        pause_mission(the_connection)
        time.sleep(3)
        hotspots = detect_hotspots_with_mask(image_path, threshold=0.7)
        print(f"Detected hotspots: {hotspots}")
        # Get the latest GPS coordinates from drone
        lat, lon, alt = get_latest_gps(the_connection)

        # Map hotspots to GPS coordinates using `get_hotspots_gps`
        if hotspots:
            distorted_points = [[x, y] for x, y in hotspots]
            pitch = 0.0  # Replace with actual pitch reading
            azimuth = 0.0  # Replace with actual azimuth reading
            gps_hotspots = get_hotspots_gps(distorted_points, lon, lat, alt, pitch, azimuth)
            for gps_point in gps_hotspots:
                detected_hotspots.append({"lat": gps_point[0], "lon": gps_point[1]})
        resume_mission(the_connection)

    # Generate KML file
    kml = simplekml.Kml()
    for point in gps_hotspots:
        kml.newpoint(name="Hotspot", coords=[(point[1], point[0])])  # Lon, Lat
    
    # Save KML file
    os.makedirs(os.path.dirname(output_kml_path), exist_ok=True)
    kml.save(output_kml_path)
    print(f"KML file saved to {output_kml_path}")
        

    # Set mode RTL
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # Base mode
        6, 0, 0, 0, 0, 0  # Custom mode for GUIDED
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)



if __name__ == "__main__":
    main()
