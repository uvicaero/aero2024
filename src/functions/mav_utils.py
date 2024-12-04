from pymavlink import mavutil

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


from pymavlink import mavutil

def send_waypoint(connection, waypoint_idx, lat, lon, alt):
    """
    Sends a waypoint to the Pixhawk as part of a mission.
    """
    try:
        target_system = 1
        target_component = 1

        # Send the waypoint
        connection.mav.mission_item_int_send(
            target_system,
            target_component,
            waypoint_idx,  # Waypoint index
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Use relative altitude
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Waypoint navigation command
            2,  # Current waypoint (1 if it's the first one, otherwise 0)
            1,  # Autocontinue to the next waypoint
            0, 0, 0, 0,  # Placeholder parameters
            lat * 1e7,  # Latitude (scaled to MAVLink format)
            lon * 1e7,  # Longitude (scaled to MAVLink format)
            alt         # Altitude
        )
        print(f"Waypoint {waypoint_idx} sent: Latitude={lat}, Longitude={lon}, Altitude={alt}")
    except Exception as e:
        print(f"Error sending waypoint {waypoint_idx}: {e}")

def send_waypoints(connection, waypoints):
    """
    Sends a sequence of waypoints to the Pixhawk.
    """
    print("Starting waypoint mission upload...")
    
    # Clear existing waypoints (optional but recommended)
    connection.mav.mission_clear_all_send(
        connection.target_system, connection.target_component
    )
    print("Cleared existing mission waypoints.")

    # Send each waypoint
    for idx, (lat, lon, alt) in enumerate(waypoints):
        send_waypoint(connection, idx, lat, lon, alt)
    
    # Send mission acknowledgment (end of mission)
    connection.mav.mission_ack_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_MISSION_ACCEPTED
    )
    print("Mission upload complete. Waypoints sent.")

def switch_to_auto(connection):
    """
    Switch the Pixhawk to AUTO mode for mission execution.
    """
    try:
        # Set the vehicle to AUTO mode
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mavutil.mavlink.MAV_MODE_AUTO_ARMED
        )
        print("Vehicle switched to AUTO mode.")
    except Exception as e:
        print(f"Error switching to AUTO mode: {e}")



