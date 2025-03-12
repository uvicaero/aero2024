import numpy as np

def merge_hotspots(hotspot_positions, merge_distance=2):
    """
    Merges clusters of nearby hotspot positions by replacing them with their average center.
    
    Parameters:
        hotspot_positions (list of tuples): List of (latitude, longitude) detected hotspots.
        merge_distance (float): Maximum distance in meters to consider two points as the same hotspot.
    
    Returns:
        list of tuples: Unique (latitude, longitude) hotspot positions after merging.
    """
    if not hotspot_positions:
        return []

    # Approximate conversion factors (meters per degree at given latitude)
    lat_factor = 111230.9531021928  # Meters per degree of latitude
    lon_factor = 71546.90282746412  # Meters per degree of longitude (approximate at this latitude)

    unique_hotspots = []  # List of merged hotspots
    unprocessed = set(range(len(hotspot_positions)))  # Keep track of unprocessed hotspots

    hotspot_positions = np.array(hotspot_positions)

    while unprocessed:
        index = unprocessed.pop()  # Take an unprocessed hotspot
        lat, lon = hotspot_positions[index]

        # Find all nearby hotspots within the merge distance
        nearby_indices = []
        cluster_lat, cluster_lon = [lat], [lon]

        for i in list(unprocessed):
            u_lat, u_lon = hotspot_positions[i]

            # Convert degrees to meters and compute distance
            lat_dist = (lat - u_lat) * lat_factor
            lon_dist = (lon - u_lon) * lon_factor
            distance = np.sqrt(lat_dist**2 + lon_dist**2)  # Euclidean distance in meters

            if distance < merge_distance:
                nearby_indices.append(i)
                cluster_lat.append(u_lat)
                cluster_lon.append(u_lon)

        # Remove processed indices from unprocessed set
        for i in nearby_indices:
            unprocessed.remove(i)

        # Compute the average position of the cluster
        avg_lat = np.mean(cluster_lat)
        avg_lon = np.mean(cluster_lon)
        unique_hotspots.append((avg_lat, avg_lon))

    return unique_hotspots

# Example input: Detected hotspots from multiple images
hotspot_positions = [
    (50.1005, -110.7381), (50.1006, -110.7380),  # Detected in one image
    (50.1004, -110.7382),  # Detected in another image (should merge with above)
    (50.1010, -110.7385), (50.1011, -110.7384),  # Another hotspot detected in multiple images
    (50.1030, -110.7400),  # Unique hotspot
    (50.1005, -110.7381),  # Another detection of the first hotspot
]

# Merge nearby hotspots into their average center
merged_hotspots = merge_hotspots(hotspot_positions)

# Print results
for i, (lat, lon) in enumerate(merged_hotspots):
    print(f"Hotspot {i+1}: Latitude = {lat:.6f}, Longitude = {lon:.6f}")
