import time
from pymavlink import mavutil
import subprocess
from shapely.geometry import Point, Polygon, LineString
from src.functions.path_generator import generate_path_custom_boundary

# Connect to the MAVLink device via USB (replace with your actual port)
the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5763')

# Wait for the first heartbeat to confirm the connection
print("Waiting for heartbeat...")
the_connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

def create_waypoint_file(filename, waypoints):
    """
    Create a MAVProxy-compatible waypoint file.
    :param filename: Name of the waypoint file to create.
    :param waypoints: List of waypoints (each as a dictionary with lat, lon, alt, command).
    """
    with open(filename, "w") as f:
        f.write("QGC WPL 110\n")
        for i, wp in enumerate(waypoints):
            f.write(f"{i}\t0\t3\t{wp['command']}\t0\t0\t0\t0\t{wp['lat']}\t{wp['lon']}\t{wp['alt']}\t1\n")


def load_waypoints_via_mavlink(waypoints):
    """
    Load waypoints directly via MAVLink.
    :param waypoints: List of waypoints (each as a dictionary with lat, lon, alt, command).
    """
    print(f"Uploading {len(waypoints)} waypoints to the autopilot...")

    # Notify autopilot of the number of waypoints
    the_connection.mav.mission_count_send(
        the_connection.target_system,
        the_connection.target_component,
        len(waypoints)
    )

    time.sleep(15)

    # # Wait for MISSION_REQUEST messages and send waypoints
    # for seq, wp in enumerate(waypoints):
    #     print(f"Waiting for MISSION_REQUEST for waypoint {seq}...")
    #     while True:
    #         msg = the_connection.recv_match(type='MISSION_REQUEST_INT', blocking=True, timeout=10)
    #         if msg and msg.seq == seq:
    #             print(f"MISSION_REQUEST_INT received for waypoint {seq}. Sending waypoint...")
    #             the_connection.mav.mission_item_int_send(
    #                 the_connection.target_system,
    #                 the_connection.target_component,
    #                 seq,  # Sequence number
    #                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame
    #                 wp['command'],  # Command type (e.g., NAV_WAYPOINT)
    #                 0,  # Current waypoint (0 for all except the current one)
    #                 1,  # Auto-continue to the next waypoint
    #                 0, 0, 0, 0,  # Unused parameters
    #                 int(wp['lat'] * 1e7),  # Latitude in scaled int
    #                 int(wp['lon'] * 1e7),  # Longitude in scaled int
    #                 int(wp['alt'] * 1000)  # Altitude in millimeters
    #             )
    #             break
    #         elif not msg:
    #             print(f"Timeout waiting for MISSION_REQUEST_INT for waypoint {seq}. Aborting upload.")
    #             return False

    # Wait for MISSION_ACK
    print("Waiting for MISSION_ACK...")
    msg = the_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if msg:
        print("Waypoints uploaded successfully.")
        return True
    else:
        print("Timeout waiting for MISSION_ACK. Waypoint upload failed.")
        return False

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


def wait_for_loiter_mode():
    """
    Wait until the drone enters LOITER mode.
    """
    print("Waiting for the drone to enter LOITER mode...")
    while True:
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            custom_mode = msg.custom_mode
            print(f"Current custom mode: {custom_mode}")
            if custom_mode == 5:  # Replace with the correct custom_mode for LOITER in your firmware
                print("Drone is now in LOITER mode.")
                break
        else:
            print("Timeout waiting for HEARTBEAT.")
            return False
        time.sleep(1)
    return True


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

def main():
    """Beginning main"""
    
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

    # boundary_coords = [
    #     (48.5185696, -123.380459),
    #     (48.5128107, -123.3800704),
    #     (48.5128391, -123.371187),
    #     (48.5186052, -123.3712)
    # ]

    boundary_polygon = Polygon(boundary_coords)

    waypoint_values = generate_path_custom_boundary(48.516087,-123.376277,20,boundary_polygon)

    print(waypoint_values)

    waypoints = []

    for value in waypoint_values:
        print("Appending {value}")
        waypoints.append({"lat": value[0], "lon": value[1], "alt": value[2], "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})

    # Set mode
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

    
    # Load waypoints
    if not load_waypoints_via_mavlink(waypoints):
        print("Failed to upload waypoints. Exiting.")
        return

    # Arm the vehicle
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # Arm
        0, 0, 0, 0, 0, 0
    )
    wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    time.sleep(5)

    # Takeoff to an altitude of 20 meters
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, float('nan'),
        0, 0,
        20
    )
    
    wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    wait_for_takeoff_completion(20)

    # Start the mission
    if not start_mission():
        print("Failed to start the mission. Exiting.")
        return

    time.sleep(5)

    # Wait for loiter mode
    wait_for_loiter_mode()


if __name__ == "__main__":
    main()
