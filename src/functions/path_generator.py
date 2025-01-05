import argparse
import plotly.graph_objects as go
from shapely.geometry import Point, Polygon, LineString
import simplekml
import math
import random

def find_nearest_waypoint(point, waypoints):
    point_geom = Point(point)
    closest_point = min(waypoints, key=lambda wpt: point_geom.distance(Point(wpt)))
    return closest_point

def generate_boundary_waypoints(boundary_polygon, step_distance=5):
    boundary_waypoints = []
    line = boundary_polygon.exterior  # Get the boundary as a LineString
    total_distance = line.length
    num_steps = 100
    
    # Add waypoints at each corner of the polygon (vertices)
    for coord in list(line.coords):
        boundary_waypoints.append((coord[0], coord[1]))  # (lon, lat) order

    # Generate waypoints along the boundary at regular intervals
    for i in range(num_steps + 1):
        interpolated_point = line.interpolate(i / num_steps, normalized=True)
        waypoint = (interpolated_point.x, interpolated_point.y)
        # Only add if it's not already in the list to avoid duplicates
        if waypoint not in boundary_waypoints:
            boundary_waypoints.append(waypoint)

    # Sort waypoints in order along the boundary
    boundary_waypoints = sorted(boundary_waypoints, key=lambda pt: line.project(Point(pt)))

    return boundary_waypoints


def create_boundary_constrained_path(start_point, end_point, boundary_polygon, step_distance=5):
    # Step 1: Generate precomputed waypoints around the boundary
    line = boundary_polygon.exterior
    boundary_waypoints = []

    # Add all corner points (vertices) of the polygon
    for coord in list(line.coords):
        waypoint = (coord[0], coord[1])
        if waypoint not in boundary_waypoints:
            boundary_waypoints.append(waypoint)

    # Generate additional waypoints along the boundary at regular intervals
    num_steps = 100
    for i in range(num_steps + 1):
        interpolated_point = line.interpolate(i / num_steps, normalized=True)
        waypoint = (interpolated_point.x, interpolated_point.y)
        if waypoint not in boundary_waypoints:
            boundary_waypoints.append(waypoint)

    # Sort waypoints by their position along the boundary
    boundary_waypoints = sorted(boundary_waypoints, key=lambda pt: line.project(Point(pt)))

    # Step 2: Find the nearest waypoints on the boundary to the start and end points
    start_nearest = min(boundary_waypoints, key=lambda wpt: Point(wpt).distance(Point(start_point.x, start_point.y)))
    end_nearest = min(boundary_waypoints, key=lambda wpt: Point(wpt).distance(Point(end_point.x, end_point.y)))

    # Step 3: Determine the indices of the nearest points in the boundary waypoints list
    start_idx = boundary_waypoints.index(start_nearest)
    end_idx = boundary_waypoints.index(end_nearest)

    # Step 4: Generate the path along the boundary, choosing the shorter traversal (clockwise or counter-clockwise)
    if start_idx <= end_idx:
        # Clockwise path: Direct from start to end
        clockwise_path = boundary_waypoints[start_idx:end_idx + 1]
        # Counter-clockwise path: Wraps around the boundary
        counter_clockwise_path = boundary_waypoints[end_idx:] + boundary_waypoints[:start_idx + 1]
    else:
        # Clockwise path: Wraps around the boundary
        clockwise_path = boundary_waypoints[start_idx:] + boundary_waypoints[:end_idx + 1]
        # Counter-clockwise path: Direct from end to start
        counter_clockwise_path = boundary_waypoints[end_idx:start_idx + 1]

    # Step 5: Choose the shorter path
    if len(clockwise_path) <= len(counter_clockwise_path):
        path = clockwise_path
    else:
        path = counter_clockwise_path


    return path


# Function to calculate a new lat/lon based on distance and bearing
def calculate_offset(lat, lon, bearing, distance):
    R = 6378137.0  # Radius of Earth in meters
    lat = math.radians(lat)
    lon = math.radians(lon)
    bearing = math.radians(bearing)

    new_lat = math.asin(math.sin(lat) * math.cos(distance / R) +
                        math.cos(lat) * math.sin(distance / R) * math.cos(bearing))
    new_lon = lon + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat),
                               math.cos(distance / R) - math.sin(lat) * math.sin(new_lat))
    
    return math.degrees(new_lat), math.degrees(new_lon)

# Define the Soft Boundary polygon coordinates from KML
green_boundary_coords = [
    (50.0971537, -110.7329257),
    (50.1060519, -110.7328869),
    (50.1060793, -110.7436756),
    (50.1035452, -110.7436555),
    (50.0989139, -110.7381534),
    (50.0971788, -110.7381487),
    (50.0971537, -110.7329257)
]

# Create a Polygon for the green boundary
green_boundary_polygon = Polygon(green_boundary_coords)


# Parameters for Archimedean Spiral with dynamic angle adjustment
# altitude = 100              # Altitude in meters
max_radius = 125            # Radius of search area in meters
initial_radius = 12          # Initial radius to start the spiral
radius_growth = 0.14         # Growth factor for each step in the angle
desired_distance = 20        # Desired consistent distance between waypoints in meters
camera_radius = 30          # Approximate coverage radius for Raspberry Pi Camera V2 at 50m height


def generate_path(center_lat, center_lon, altitude):

    # Main loop for creating waypoints and adding oriented squares
    previous_lat, previous_lon = None, None  # Initialize previous waypoint

    # Track the last in-bound waypoint and out-of-bound flag
    previous_inbound_lat, previous_inbound_lon = None, None
    out_of_bounds_flag = False

    # Generate waypoints using the Archimedean Spiral pattern for this center point
    waypoints = []
    angle = 0
    radius = initial_radius


    while radius <= max_radius:
        lat, lon = calculate_offset(center_lat, center_lon, angle, radius)
        print(f"Point (current): Latitude = {lat}, Longitude = {lon}\n\n")
        if green_boundary_polygon.contains(Point(lat, lon)):
            print(f"Point (current in-bound): Latitude = {lat}, Longitude = {lon}\n\n")
            # If we re-enter the boundary and the out-of-bounds flag is set
            if out_of_bounds_flag and previous_inbound_lat is not None and previous_inbound_lon is not None:
                # Generate boundary-following waypoints from previous in-bound point to current point
                print("Re-entering boundary. Calling create_boundary_waypoints...\n\n")
                start_point = Point(previous_inbound_lat, previous_inbound_lon)
                end_point = Point(lat, lon)
                print(f"Start Point (last in-bound): Latitude = {previous_inbound_lat}, Longitude = {previous_inbound_lon}\n\n")
                print(f"End Point (current in-bound): Latitude = {lat}, Longitude = {lon}\n\n")
                boundary_waypoints = create_boundary_constrained_path(start_point, end_point, green_boundary_polygon)
                print("Boundary waypoints generated:")
                for idx, (wpt_lat, wpt_lon) in enumerate(boundary_waypoints):
                    print(f"Boundary Waypoint {idx + 1}: Latitude = {wpt_lat}, Longitude = {wpt_lon}")

                
                # Add each boundary-following waypoint to the waypoints list in reverse order
                for wpt_lat, wpt_lon in reversed(boundary_waypoints):
                    waypoints.append((wpt_lat, wpt_lon, altitude))
                
            # Reset the out-of-bounds flag
            out_of_bounds_flag = False

            waypoints.append((lat, lon, altitude))

                # Update previous waypoint
            previous_lat, previous_lon = lat, lon
            previous_inbound_lat, previous_inbound_lon = lat, lon
        else:
            # Set the out-of-bounds flag and continue to the next iteration
            out_of_bounds_flag = True

        
        # Calculate angle increment based on radius for consistent waypoint spacing
        angle_increment = math.degrees(desired_distance / radius)  # Convert radians to degrees
        print(f"Iteration: Radius = {radius:.2f} m, Angle Increment = {angle_increment:.2f} degrees")
        angle += angle_increment
        radius = initial_radius + radius_growth * angle

    return waypoints

def generate_path_custom_boundary(center_lat, center_lon, altitude, boundary_polygon):

    # Main loop for creating waypoints and adding oriented squares
    previous_lat, previous_lon = None, None  # Initialize previous waypoint

    # Track the last in-bound waypoint and out-of-bound flag
    previous_inbound_lat, previous_inbound_lon = None, None
    out_of_bounds_flag = False

    # Generate waypoints using the Archimedean Spiral pattern for this center point
    waypoints = []
    angle = 0
    radius = initial_radius


    while radius <= max_radius:
        lat, lon = calculate_offset(center_lat, center_lon, angle, radius)
        print(f"Point (current): Latitude = {lat}, Longitude = {lon}\n\n")
        if boundary_polygon.contains(Point(lat, lon)):
            print(f"Point (current in-bound): Latitude = {lat}, Longitude = {lon}\n\n")
            # If we re-enter the boundary and the out-of-bounds flag is set
            if out_of_bounds_flag and previous_inbound_lat is not None and previous_inbound_lon is not None:
                # Generate boundary-following waypoints from previous in-bound point to current point
                #print("Re-entering boundary. Calling create_boundary_waypoints...\n\n")
                start_point = Point(previous_inbound_lat, previous_inbound_lon)
                end_point = Point(lat, lon)
                #print(f"Start Point (last in-bound): Latitude = {previous_inbound_lat}, Longitude = {previous_inbound_lon}\n\n")
                #print(f"End Point (current in-bound): Latitude = {lat}, Longitude = {lon}\n\n")
                boundary_waypoints = create_boundary_constrained_path(start_point, end_point, boundary_polygon)
                #print("Boundary waypoints generated:")
                #for idx, (wpt_lat, wpt_lon) in enumerate(boundary_waypoints):
                    #print(f"Boundary Waypoint {idx + 1}: Latitude = {wpt_lat}, Longitude = {wpt_lon}")

                
                # Add each boundary-following waypoint to the waypoints list in reverse order
                for wpt_lat, wpt_lon in reversed(boundary_waypoints):
                    waypoints.append((wpt_lat, wpt_lon, altitude))
                
            # Reset the out-of-bounds flag
            out_of_bounds_flag = False
            print("Appending waypoint")
            waypoints.append((lat, lon, altitude))

                # Update previous waypoint
            previous_lat, previous_lon = lat, lon
            previous_inbound_lat, previous_inbound_lon = lat, lon
        else:
            # Set the out-of-bounds flag and continue to the next iteration
            print(f"Point is OUTSIDE the boundary: lat={lat}, lon={lon}")
            out_of_bounds_flag = True

        
        # Calculate angle increment based on radius for consistent waypoint spacing
        angle_increment = math.degrees(desired_distance / radius)  # Convert radians to degrees
        #print(f"Iteration: Radius = {radius:.2f} m, Angle Increment = {angle_increment:.2f} degrees")
        angle += angle_increment
        radius = initial_radius + radius_growth * angle
    return waypoints

def generate_path_custom_boundary_custom_radii(center_lat, center_lon, altitude, boundary_polygon, custom_start_radius, custom_max_radius, custom_desired_distance, custom_radius_growth, custom_radius_factor):

    # Main loop for creating waypoints and adding oriented squares
    previous_lat, previous_lon = None, None  # Initialize previous waypoint

    # Track the last in-bound waypoint and out-of-bound flag
    previous_inbound_lat, previous_inbound_lon = None, None
    out_of_bounds_flag = False

    # Generate waypoints using the Archimedean Spiral pattern for this center point
    waypoints = []
    angle = 0
    radius = custom_start_radius


    while radius <= custom_max_radius:
        lat, lon = calculate_offset(center_lat, center_lon, angle, radius)
        print(f"Point (current): Latitude = {lat}, Longitude = {lon}\n\n")
        if boundary_polygon.contains(Point(lat, lon)):
            print(f"Point (current in-bound): Latitude = {lat}, Longitude = {lon}\n\n")
            # If we re-enter the boundary and the out-of-bounds flag is set
            if out_of_bounds_flag and previous_inbound_lat is not None and previous_inbound_lon is not None:
                # Generate boundary-following waypoints from previous in-bound point to current point
                #print("Re-entering boundary. Calling create_boundary_waypoints...\n\n")
                start_point = Point(previous_inbound_lat, previous_inbound_lon)
                end_point = Point(lat, lon)
                #print(f"Start Point (last in-bound): Latitude = {previous_inbound_lat}, Longitude = {previous_inbound_lon}\n\n")
                #print(f"End Point (current in-bound): Latitude = {lat}, Longitude = {lon}\n\n")
                boundary_waypoints = create_boundary_constrained_path(start_point, end_point, boundary_polygon)
                #print("Boundary waypoints generated:")
                #for idx, (wpt_lat, wpt_lon) in enumerate(boundary_waypoints):
                    #print(f"Boundary Waypoint {idx + 1}: Latitude = {wpt_lat}, Longitude = {wpt_lon}")

                
                # Add each boundary-following waypoint to the waypoints list in reverse order
                for wpt_lat, wpt_lon in reversed(boundary_waypoints):
                    waypoints.append((wpt_lat, wpt_lon, altitude))
                
            # Reset the out-of-bounds flag
            out_of_bounds_flag = False
            print("Appending waypoint")
            waypoints.append((lat, lon, altitude))

                # Update previous waypoint
            previous_lat, previous_lon = lat, lon
            previous_inbound_lat, previous_inbound_lon = lat, lon
        else:
            # Set the out-of-bounds flag and continue to the next iteration
            print(f"Point is OUTSIDE the boundary: lat={lat}, lon={lon}")
            out_of_bounds_flag = True

        print(radius)
        print(custom_desired_distance)
        # Calculate angle increment based on radius for consistent waypoint spacing
        angle_increment = math.degrees(custom_desired_distance / radius)  # Convert radians to degrees
        print(angle_increment)
        #print(f"Iteration: Radius = {radius:.2f} m, Angle Increment = {angle_increment:.2f} degrees")
        angle += angle_increment
        radius = custom_start_radius + custom_radius_growth * (angle/custom_radius_factor)
        print(angle)
        print(radius)
    return waypoints