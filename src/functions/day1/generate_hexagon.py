import numpy as np

def generate_hexagon_gps(start_lat, start_lon, spacing=75):
    """
    Generates a hexagonal flight path in GPS coordinates.

    Parameters:
        start_lat (float): Starting latitude in degrees (hexagon center).
        start_lon (float): Starting longitude in degrees (hexagon center).
        spacing (float): Distance from the center to each vertex in meters.

    Returns:
        waypoints (dict): Dictionary of labeled hexagon vertices.
    """
    # Define angles for a regular hexagon (60-degree increments)
    angles = np.linspace(0, 2 * np.pi, 7)[:-1]  # 6 points + closing the loop

    # Approximate conversion factors (meters per degree)
    lat_factor = 1 / 111320  # Degrees latitude per meter
    lon_factor = 1 / (111320 * np.cos(np.radians(start_lat)))  # Degrees longitude per meter

    waypoints = {}
    for i, angle in enumerate(angles):
        # Convert meters to degrees and calculate new coordinates
        delta_lat = (spacing * np.sin(angle)) * lat_factor
        delta_lon = (spacing * np.cos(angle)) * lon_factor

        new_lat = start_lat + delta_lat
        new_lon = start_lon + delta_lon

        waypoints[f"Vertex_{i+1}"] = (new_lat, new_lon)

    # Close the hexagon loop
    waypoints["Vertex_7"] = waypoints["Vertex_1"]

    return waypoints

# Example usage
start_lat = 50.1010  # Example start latitude (center of hexagon)
start_lon = -110.7380  # Example start longitude (center of hexagon)
hexagon_waypoints = generate_hexagon_gps(start_lat, start_lon, spacing=75)

# Print results
for label, (lat, lon) in hexagon_waypoints.items():
    print(f"{label}: Latitude = {lat:.6f}, Longitude = {lon:.6f}")
