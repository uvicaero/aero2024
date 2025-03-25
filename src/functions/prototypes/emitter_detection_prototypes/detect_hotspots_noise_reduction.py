import math
import numpy as np
import cv2
from src.functions.prototypes.emitter_detection_prototypes.detect_hotspots_with_mask import detect_hotspots_with_mask
from src.functions.get_hotspots_gps import get_hotspots_gps

# Test Accuracy of GPS mapping with simulated test values and single sample image
def main():

    image_path = "data/field_data/ir_detection_test_images/aerial3.jpg"
    
    # Initialize hotspots list
    detected_hotspots = []
    get_gps_points = []
    # Todo Find coordinates to test
    LAT = 1
    LON = 1
    ALT = 1
    YAW = 1

    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    hotspots = detect_hotspots_with_mask(image, threshold=0.7) ############ UPDATE detect hotspot fxn
    print(f"Detected hotspots: {hotspots}")

    # Get the latest GPS coordinates from drone
    get_gps_points.append({"lat": LAT, "lon": LON})

    # Map hotspots to GPS coordinates using `get_hotspots_gps`
    if hotspots.size > 0:
        distorted_points = [[x, y] for x, y in hotspots]
        dist_pts = np.array(distorted_points, dtype=np.float32)
        pitch = math.radians(-90)
        azimuth = YAW  # Replace with actual azimuth reading
        gps_hotspots = get_hotspots_gps(dist_pts, LON, LAT, ALT, pitch, azimuth)

        # Fix shape if necessary: Convert (N, 1, 2) ? (N, 2)
        if gps_hotspots.ndim == 3 and gps_hotspots.shape[1] == 1 and gps_hotspots.shape[2] == 2:
            gps_hotspots = gps_hotspots.reshape(-1, 2)

        # Ensure valid data before appending
        if gps_hotspots.size > 0 and gps_hotspots.shape[1] == 2:
            for gps_point in gps_hotspots:
                detected_hotspots.append({"lat": gps_point[0], "lon": gps_point[1]})
        else:
            print(f"Error: Invalid GPS hotspots output {gps_hotspots}")


    # Todo Find expected coordinates based on input
    expected_lon = 1
    expected_lat = 1
    print(f"Expected Coordinates: {expected_lon}, {expected_lat}")
    for point in detected_hotspots:
        print(f"Calculated Coordinates: {point['lon']}, {point['lat']}")

if __name__ == "__main__":
    main()
