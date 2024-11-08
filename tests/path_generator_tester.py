import argparse
import plotly.graph_objects as go
from shapely.geometry import Point, Polygon
import simplekml
import random
import math
from pathlib import Path

# Define the output directory relative to the root of the project
project_root = Path(__file__).resolve().parent.parent
output_dir = project_root / 'data' / 'test_outputs' / 'path_generator_tester_kml'
output_dir.mkdir(parents=True, exist_ok=True)

# Define the output file path
output_file = output_dir / 'tester_output.kml'


from src.functions.path_generator import generate_path, green_boundary_polygon, calculate_offset

# Parameters for the search area
altitude = 100
max_radius = 125
initial_radius = 12
radius_growth = 0.14
desired_distance = 20

def get_unique_filename(output_dir, base_name="tester_output", extension=".kml"):
    index = 1
    while True:
        filename = f"{base_name}_{index}{extension}"
        output_file = output_dir / filename
        if not output_file.exists():
            return output_file
        index += 1

# Function to calculate bearing between two points (in degrees)
def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    return math.degrees(initial_bearing) % 360

# Function to generate a random point within the boundary polygon using centroid with random shift
def random_point_within_boundary(boundary_polygon):
    center_point = boundary_polygon.centroid
    while True:
        lat_shift = random.uniform(-0.0150, 0.0150)
        lon_shift = random.uniform(-0.0150, 0.0150)
        random_point = Point(center_point.x + lat_shift, center_point.y + lon_shift)
        if boundary_polygon.contains(random_point):
            return random_point.x, random_point.y

# Function to add a 100m circle around the center point in KML
def add_circle_to_kml(kml, lat, lon, radius, altitude, name, color):
    circle = kml.newpolygon(name=name)
    coords = []
    num_points = 36  # Approximation of the circle with 36 points
    for i in range(num_points):
        angle = i * (360 / num_points)
        point_lat, point_lon = calculate_offset(lat, lon, angle, radius)
        coords.append((point_lon, point_lat, altitude))
    circle.outerboundaryis = coords
    circle.style.polystyle.color = color
    circle.style.polystyle.fill = 1
    circle.style.polystyle.outline = 0

# Function to add an oriented square around each waypoint in KML
def add_oriented_square_to_kml(kml, lat, lon, width, height, altitude, bearing, name, color):
    half_width = width / 2
    half_height = height / 2

    # Calculate the four corners of the square, rotated by the bearing
    top_left = calculate_offset(lat, lon, bearing - 135, math.sqrt(half_width**2 + half_height**2))
    top_right = calculate_offset(lat, lon, bearing - 45, math.sqrt(half_width**2 + half_height**2))
    bottom_right = calculate_offset(lat, lon, bearing + 45, math.sqrt(half_width**2 + half_height**2))
    bottom_left = calculate_offset(lat, lon, bearing + 135, math.sqrt(half_width**2 + half_height**2))

    # Create polygon for the square
    square = kml.newpolygon(name=name)
    square.outerboundaryis = [
        (top_left[1], top_left[0], altitude),
        (top_right[1], top_right[0], altitude),
        (bottom_right[1], bottom_right[0], altitude),
        (bottom_left[1], bottom_left[0], altitude),
        (top_left[1], top_left[0], altitude)  # Close the loop
    ]
    square.style.polystyle.color = color
    square.style.polystyle.fill = 1
    square.style.polystyle.outline = 0

# Function to save waypoints and features to a combined KML file
def save_to_kml(all_waypoints, centers, filename="combined_paths_with_coverage.kml"):
    kml = simplekml.Kml()

    # Add the boundary polygon
    boundary_polygon = kml.newpolygon(name="Boundary")
    boundary_polygon.outerboundaryis = [(lon, lat) for lat, lon in green_boundary_polygon.exterior.coords]
    boundary_polygon.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.green)
    boundary_polygon.style.polystyle.fill = 1
    boundary_polygon.style.polystyle.outline = 1

    # Add paths, circles, and coverage areas for each center point
    for index, (waypoints, (center_lat, center_lon)) in enumerate(zip(all_waypoints, centers)):
        # Add the 100m circle around the center point
        add_circle_to_kml(
            kml,
            center_lat,
            center_lon,
            100,
            altitude,
            f"Center Boundary {index + 1}",
            simplekml.Color.changealphaint(100, simplekml.Color.blue)
        )

        # Add the waypoints and oriented squares
        for idx, (lat, lon, alt) in enumerate(waypoints):
            pnt = kml.newpoint(name=f"Point {index + 1} - Waypoint {idx + 1}", coords=[(lon, lat, alt)])
            pnt.style.iconstyle.color = simplekml.Color.red

            # Calculate bearing to the next waypoint (or default to north if last waypoint)
            if idx < len(waypoints) - 1:
                next_lat, next_lon, _ = waypoints[idx + 1]
                bearing = calculate_bearing(lat, lon, next_lat, next_lon)
            else:
                bearing = 0  # Default to north for the last waypoint

            # Add an oriented square around the waypoint
            add_oriented_square_to_kml(
                kml,
                lat,
                lon,
                63.2,  # Width of the camera coverage area in meters
                47.2,  # Height of the camera coverage area in meters
                altitude,
                bearing,
                f"Coverage Box {idx + 1}",
                simplekml.Color.changealphaint(100, simplekml.Color.red)
            )

    # Save the KML file
    kml.save(str(filename))
    print(f"KML file saved to: {filename}")

# Function to plot all waypoints using Plotly
def plot_all_waypoints(all_waypoints, centers, boundary_coords):
    fig = go.Figure()

    # Plot the boundary
    boundary_x, boundary_y = zip(*[(lon, lat) for lat, lon in boundary_coords])
    fig.add_trace(go.Scatter(x=boundary_x, y=boundary_y, mode='lines+markers', name='Boundary'))

    # Plot the center points and waypoints for each set
    for index, (waypoints, (center_lat, center_lon)) in enumerate(zip(all_waypoints, centers)):
        # Plot the center point
        fig.add_trace(go.Scatter(x=[center_lon], y=[center_lat], mode='markers', name=f'Center {index + 1}', marker=dict(size=10, color='blue')))

        # Plot the waypoints
        waypoint_x = [lon for _, lon, _ in waypoints]
        waypoint_y = [lat for lat, _, _ in waypoints]
        fig.add_trace(go.Scatter(x=waypoint_x, y=waypoint_y, mode='markers+lines', name=f'Waypoints for Point {index + 1}'))

    # Update the layout
    fig.update_layout(
        title="Generated Paths with Coverage Areas for All Sets",
        xaxis_title="Longitude",
        yaxis_title="Latitude"
    )

    # Show the plot
    fig.show()

def main():
    all_waypoints = []
    centers = []

    # Generate paths for 5 random points
    for i in range(5):
        print(f"Generating path for random point {i + 1}...")
        center_lat, center_lon = random_point_within_boundary(green_boundary_polygon)
        centers.append((center_lat, center_lon))

        # Generate the path using the function
        waypoints = generate_path(center_lat, center_lon)
        all_waypoints.append(waypoints)

        # Print the waypoints
        print(f"\nWaypoints for Point {i + 1}:")
        for idx, (lat, lon, alt) in enumerate(waypoints):
            print(f"Waypoint {idx + 1}: Latitude = {lat:.6f}, Longitude = {lon:.6f}, Altitude = {alt:.2f} m")

    # Get a unique output file path
    output_file = get_unique_filename(output_dir)

    save_to_kml(all_waypoints,centers,output_file)

    # Plot all the waypoints and the boundary
    plot_all_waypoints(all_waypoints, centers, green_boundary_polygon.exterior.coords)

if __name__ == "__main__":
    main()
