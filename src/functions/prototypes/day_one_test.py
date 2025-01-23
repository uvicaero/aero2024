import time
from pymavlink import mavutil
import threading
import subprocess
from shapely.geometry import Point, Polygon, LineString
from src.functions.path_generator import generate_path_custom_boundary, generate_path_custom_boundary_custom_radii

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
the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5763')
# the_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for the first heartbeat to confirm the connection
print("Waiting for heartbeat...")
the_connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# Start the heartbeat thread
heartbeat_thread = threading.Thread(target=send_heartbeat, args=(the_connection,), daemon=True)
heartbeat_thread.start()

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

def load_waypoints_via_mavlink(waypoints):
    """
    Load waypoints directly via MAVLink without waiting for individual ACKs.
    :param waypoints: List of waypoints (each as a dictionary with lat, lon, alt, command).
    """
    print(f"Uploading {len(waypoints)} waypoints to the autopilot...")

    # Notify autopilot of the number of waypoints
    the_connection.mav.mission_count_send(
        the_connection.target_system,
        the_connection.target_component,
        len(waypoints)
    )

    # Immediately send all waypoints sequentially without waiting for ACKs
    for seq, wp in enumerate(waypoints):
        print(f"Sending waypoint {seq}...")
        the_connection.mav.mission_item_int_send(
            the_connection.target_system,
            the_connection.target_component,
            seq,  # Sequence number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame
            wp['command'],  # Command type (e.g., NAV_WAYPOINT)
            0,  # Current waypoint (0 for all except the current one)
            1,  # Auto-continue to the next waypoint
            0, 0, 0, 0,  # Unused parameters
            int(wp['lat'] * 1e7),  # Latitude in scaled int
            int(wp['lon'] * 1e7),  # Longitude in scaled int
            int(wp['alt'])  # Altitude in millimeters
        )

    print("All waypoints sent. Waiting for MISSION_ACK...")

    # Wait for the final MISSION_ACK from the autopilot
    msg = the_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if msg:
        print("Waypoints uploaded successfully.")
        return True
    else:
        print("Timeout waiting for MISSION_ACK. Waypoint upload failed.")
        return False

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


def main():
    """Main function for uploading and executing waypoints"""
    # Define the waypoints
    old_waypoints = [
        {"lat": 48.516659, "lon": -123.375848, "alt": 20, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT},  # Waypoint
        {"lat": 48.516659, "lon": -123.375848, "alt": 20, "command": mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM},  # Loiter
    ]

    boundary_coords = [
        (48.5168217, -123.3756456),
        (48.5168892, -123.3761606),
        (48.5167258, -123.3764235),
        (48.5158658, -123.3771691),
        (48.5156384, -123.3771423),
        (48.5155212, -123.3769653),
        (48.5154181, -123.3766273),
        (48.5154465, -123.3763698),
        (48.515578, -123.3761552),
        (48.5164486, -123.3754364),
        (48.5166796, -123.3755008)
    ]

    boundary_polygon = Polygon(boundary_coords)

    current_lat, current_lon, current_alt = get_latest_gps(the_connection)

    # spiral_waypoint_values = generate_path_custom_boundary(48.516066,-123.376373,20,boundary_polygon)
    spiral_waypoint_values = generate_path_custom_boundary_custom_radii(current_lat,current_lon,20,boundary_polygon,1,20,5,0.15,6)

    print(spiral_waypoint_values)

    waypoints = []

    for value in spiral_waypoint_values:
        print(f"Appending {value}")
        waypoints.append({"lat": value[0], "lon": value[1], "alt": value[2], "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})


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

    # Start the mission
    if not start_mission():
        print("Failed to start the mission. Exiting.")
        return

    time.sleep(5)

    if wait_for_waypoint(the_connection, len(waypoints)-1):
        print(f"Final Waypoint reached successfully!")
    else:
        print(f"Failed to reach final waypoint")

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
