import time
from pymavlink import mavutil

# Connect to the MAVLink device via USB (replace with your actual port)
mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5763')

# Wait for the first heartbeat to confirm the connection
print("Waiting for heartbeat...")
mavlink_connection.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (mavlink_connection.target_system, mavlink_connection.target_component))

def monitor_messages(msg_type):
    """Monitor and display specific MAVLink messages."""
    try:
        print(f"Monitoring {msg_type} messages. Press Ctrl+C to stop.")
        while True:
            msg = mavlink_connection.recv_match(type=msg_type, blocking=True)
            if msg:
                print(msg)
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
        time.sleep(1)

def send_command(command):
    """Send MAVLink command to the system."""
    try:
        if command == "ARM":
            mavlink_connection.mav.command_long_send(
                mavlink_connection.target_system,
                mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            print("Sent ARM command.")

            # Wait for acknowledgment
            ack = mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("ARM command acknowledged.")
                else:
                    print(f"ARM command not accepted. Result: {ack.result}")
            else:
                print("No acknowledgment received for ARM command.")

        elif command == "DISARM":
            mavlink_connection.mav.command_long_send(
                mavlink_connection.target_system,
                mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            print("Sent DISARM command.")

            # Wait for acknowledgment
            ack = mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("DISARM command acknowledged.")
                else:
                    print(f"DISARM command not accepted. Result: {ack.result}")
            else:
                print("No acknowledgment received for DISARM command.")

        else:
            print(f"Command '{command}' not recognized.")
    except Exception as e:
        print(f"Failed to send command: {e}")

def main():
    """Main function to provide a terminal interface."""
    while True:
        print("\nSelect an option:")
        print("1. Monitor messages")
        print("2. Send MAVLink command")
        print("3. Exit")
        choice = input("Enter your choice: ")

        if choice == "1":
            msg_type = input("Enter the MAVLink message type to monitor (e.g., ATTITUDE): ")
            monitor_messages(msg_type)
        elif choice == "2":
            command = input("Enter the command (e.g., ARM, DISARM): ").strip().upper()
            send_command(command)
        elif choice == "3":
            print("Exiting.")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()
