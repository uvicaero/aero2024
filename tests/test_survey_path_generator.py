import os
import pytest
import math
import random
import simplekml
import plotly.graph_objects as go
from shapely.geometry import Point, Polygon
from pathlib import Path
from src.functions.path_generator import (
    generate_path,
    green_boundary_polygon,
    calculate_offset,
)

# Constants
altitude = 100
output_dir = Path(__file__).resolve().parent.parent / 'data' / 'test_outputs' / 'path_generator_tester_kml'
output_dir.mkdir(parents=True, exist_ok=True)

# Helper function: Generate a unique filename
def get_unique_filename(output_dir, base_name="tester_output", extension=".kml"):
    index = 1
    while True:
        filename = f"{base_name}_{index}{extension}"
        output_file = output_dir / filename
        if not output_file.exists():
            return output_file
        index += 1

# Helper function: Calculate bearing between two points
def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    return math.degrees(initial_bearing) % 360

# Helper function: Generate a random point within the boundary
def random_point_within_boundary(boundary_polygon):
    center_point = boundary_polygon.centroid
    while True:
        lat_shift = random.uniform(-0.0150, 0.0150)
        lon_shift = random.uniform(-0.0150, 0.0150)
        random_point = Point(center_point.x + lat_shift, center_point.y + lon_shift)
        if boundary_polygon.contains(random_point):
            return random_point.x, random_point.y

# Helper function: Add a circle to KML
def add_circle_to_kml(kml, lat, lon, radius, altitude, name, color):
    circle = kml.newpolygon(name=name)
    coords = []
    num_points = 36
    for i in range(num_points):
        angle = i * (360 / num_points)
        point_lat, point_lon = calculate_offset(lat, lon, angle, radius)
        coords.append((point_lon, point_lat, altitude))
    circle.outerboundaryis = coords
    circle.style.polystyle.color = color
    circle.style.polystyle.fill = 1
    circle.style.polystyle.outline = 0

# Helper function: Save waypoints and features to a KML file
def save_to_kml(all_waypoints, centers, filename):
    kml = simplekml.Kml()
    for index, (waypoints, (center_lat, center_lon)) in enumerate(zip(all_waypoints, centers)):
        add_circle_to_kml(kml, center_lat, center_lon, 100, altitude, f"Center Boundary {index + 1}", simplekml.Color.blue)
        for lat, lon, alt in waypoints:
            pnt = kml.newpoint(coords=[(lon, lat, alt)])
            pnt.style.iconstyle.color = simplekml.Color.red
    kml.save(str(filename))

# Helper function: Plot all waypoints using Plotly
def plot_all_waypoints(all_waypoints, centers, boundary_coords, output_file=None):
    fig = go.Figure()
    boundary_x, boundary_y = zip(*[(lon, lat) for lat, lon in boundary_coords])
    fig.add_trace(go.Scatter(x=boundary_x, y=boundary_y, mode='lines', name='Boundary'))

    for index, (waypoints, (center_lat, center_lon)) in enumerate(zip(all_waypoints, centers)):
        fig.add_trace(go.Scatter(x=[center_lon], y=[center_lat], mode='markers', name=f'Center {index + 1}', marker=dict(size=10)))
        waypoint_x = [lon for _, lon, _ in waypoints]
        waypoint_y = [lat for lat, _, _ in waypoints]
        fig.add_trace(go.Scatter(x=waypoint_x, y=waypoint_y, mode='markers+lines', name=f'Waypoints {index + 1}'))

    if output_file:
        fig.write_image(output_file)
    else:
        fig.show()

# Test case: Generate KML file
def test_generate_kml():
    all_waypoints = []
    centers = []

    # Generate paths for 3 random points
    for i in range(3):
        center_lat, center_lon = random_point_within_boundary(green_boundary_polygon)
        centers.append((center_lat, center_lon))
        waypoints = generate_path(center_lat, center_lon, altitude)
        all_waypoints.append(waypoints)

    # Generate a unique output file
    output_file = get_unique_filename(output_dir / "kml", base_name="generated_kml", extension=".kml")
    save_to_kml(all_waypoints, centers, output_file)

    # Verify that the KML file was created
    assert output_file.exists(), f"KML file was not created: {output_file}"

# Test case: Generate plot
@pytest.mark.skipif(not os.getenv("RUN_PLOT_TESTS"), reason="Plotting test skipped by default for manual inspection")
def test_generate_plot():
    all_waypoints = []
    centers = []

    # Generate paths for 3 random points
    for i in range(3):
        center_lat, center_lon = random_point_within_boundary(green_boundary_polygon)
        centers.append((center_lat, center_lon))
        waypoints = generate_path(center_lat, center_lon, altitude)
        all_waypoints.append(waypoints)

    # Generate a plot and save it to a file
    output_file = get_unique_filename(output_dir/"plots", base_name="generated_plot", extension=".png")
    plot_all_waypoints(all_waypoints, centers, green_boundary_polygon.exterior.coords, output_file)

    # Verify that the plot image was created
    assert output_file.exists(), f"Plot image was not created: {output_file}"
