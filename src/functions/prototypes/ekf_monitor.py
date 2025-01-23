from pymavlink import mavutil

# Define the bitmask for EKF_PRED_POS_HORIZ_ABS
EKF_PRED_POS_HORIZ_ABS_BIT = 1 << 6  # Bit 6

def monitor_ekf_status(mavlink_connection):
    print("Monitoring EKF_STATUS_REPORT messages...")
    while True:
        # Wait for the next message
        msg = mavlink_connection.recv_match(type="EKF_STATUS_REPORT", blocking=True)
        if not msg:
            continue

        # Check if the EKF_PRED_POS_HORIZ_ABS bit is set in the flags
        if msg.flags & EKF_PRED_POS_HORIZ_ABS_BIT:
            print("EKF_PRED_POS_HORIZ_ABS bit is set!")
            # You can add additional actions here, such as sending an alert or logging.

if __name__ == "__main__":
    # Connect to the MAVLink device
    mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

    # Wait for a heartbeat to confirm the connection
    print("Waiting for heartbeat...")
    mavlink_connection.wait_heartbeat()
    print("Heartbeat received. Connection established.")

    # Start monitoring EKF_STATUS_REPORT messages
    monitor_ekf_status(mavlink_connection)
