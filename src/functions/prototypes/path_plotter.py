import plotly.graph_objects as go
from shapely.geometry import Polygon, Point
import math
from src.functions.path_generator import generate_path_custom_boundary, generate_path_custom_boundary_custom_radii


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


# Function to plot the results
def plot_waypoints_with_boundary(boundary_coords, waypoints):
    fig = go.Figure()

    # Plot boundary polygon
    boundary_x, boundary_y = zip(*boundary_coords)
    fig.add_trace(go.Scatter(
        x=list(boundary_x) + [boundary_x[0]],  # Close the polygon
        y=list(boundary_y) + [boundary_y[0]],
        mode='lines',
        name='Boundary',
        line=dict(color='red', width=2)
    ))

    # Plot waypoints
    if waypoints:
        waypoint_x, waypoint_y = zip(*[(wp[1], wp[0]) for wp in waypoints])  # Longitude, Latitude
        fig.add_trace(go.Scatter(
            x=waypoint_x,
            y=waypoint_y,
            mode='markers+lines',
            name='Spiral Waypoints',
            marker=dict(color='blue', size=6)
        ))

    # Configure layout
    fig.update_layout(
        title="Generated Spiral Path with Boundary",
        xaxis_title="Longitude",
        yaxis_title="Latitude",
        showlegend=True,
        template="plotly_white",
        xaxis=dict(scaleanchor="y", scaleratio=1),  # Equal axis scaling
        yaxis=dict(scaleanchor="x", scaleratio=1)
    )
    fig.show()


# Main program
if __name__ == "__main__":
    # Define the boundary coordinates (example data)
    green_boundary_coords = [
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

    # Create the boundary polygon
    green_boundary_polygon = Polygon(green_boundary_coords)

    # Generate waypoints
    center_lat = 48.516087
    center_lon = -123.376277
    altitude = 20
    custom_start_radius = 1
    custom_max_radius = 20
    custom_desired_distance = 5
    custom_radius_growth = 0.15
    custom_radius_factor = 6

    waypoints = generate_path_custom_boundary_custom_radii(
        center_lat, center_lon, altitude, green_boundary_polygon,
        custom_start_radius, custom_max_radius,
        custom_desired_distance, custom_radius_growth,
        custom_radius_factor
    )

    # Plot the results
    plot_waypoints_with_boundary(green_boundary_coords, waypoints)
