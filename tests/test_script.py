
from pymavlink import mavutil
#from src.functions.mav_utils import get_latest_gps, send_waypoint, send_waypoints, set_auto_mode

def run():
    print("Running...")
    # connect to SITL
    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    #tcp:127.0.0.1:5762
    #udpin:localhost:14540

    # wait for a heartbeat
    connection.wait_heartbeat()

    # inform user
    print("Connected to system:", connection.target_system, ", component:", connection.target_component)

    print("Connected: ")
    print(connection)

    # Get current location
    #lat, lon, alt = get_latest_gps(connection)

    #print("Alt:")
    #print(alt)

if __name__ == "__main__":
    print("Test1")
    run()
