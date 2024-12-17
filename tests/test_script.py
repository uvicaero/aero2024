
from pymavlink import mavutil
from src.functions.mav_utils import get_latest_gps, send_waypoint, send_waypoints, switch_to_auto
import time

def run():
    print("Running...")
    # connect to SITL over tcp:127.0.0.1:5762
    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5763')

    # wait for a heartbeat
    connection.wait_heartbeat()

    # inform user
    print("Connected to system:", connection.target_system, ", component:", connection.target_component)

    print("Connected: ")
    print(connection)
    while True:
        print(get_latest_gps(connection))
        time.sleep(2.5)
    # while True:
    #     # Wait for the next MAVLink message
    #     message = connection.recv_match(blocking=True, timeout=1)
        
    #     if message:
    #         # Print the message
    #         print(message.to_dict())
    #     else:
    #         print("No message received, retrying...")


    # Get current location
    #lat, lon, alt = get_latest_gps(connection)

    #print("Alt:")
    #print(alt)

if __name__ == "__main__":
    print("Test1")
    run()
