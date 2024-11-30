"""1. Sanity check: assert that the correct number of estimates are always generated given a random number of inputs
    2. Correct region: Given a center point and the coverage area of the camera 
    from the specified altitude, input random hotspot locations and confirm that
      all the generated estimates are within the region the camera can see
      - for now, for simplicity, assume camera is pointing straight down and check for within a circle
      - of radius 1/2-observed-area's-length. that way don't need to worry about azimuth.
    3. Center point testing: Given a hotspot in the center of the screen, confirm that
      the guess is always the same as the vehicle location when the camera is pointing down
    4. check that the direction of the point from centre with azi applied is the same as the direction of output gps?
"""
import numpy as np
import random
import traceback
import math
from src.functions.get_hotspots_gps import (get_hotspots_gps, 
                                            lat_factor, long_factor,
                                            focal_length, sensor_width, sensor_height)


# Helper function: generate n random points
def generate_args():

    num_points = random.randint(1,10)
    rng = np.random.default_rng()

    xpoints = rng.uniform(low=0, high=3280, size=(num_points,1))
    ypoints = rng.uniform(low=0, high=2464, size=(num_points,1))
    points = np.hstack((xpoints, ypoints))

    cam_x = 50.1 + random.uniform(-0.1, 0.1)
    cam_y = -110.7 + random.uniform(-0.1, 0.1)
    altitude = random.uniform(1, 1000)
    pitch = -1 * random.uniform(0, 3.14/2)
    azimuth = random.uniform(0, 6.283)

    return points, num_points, cam_x, cam_y, altitude, pitch, azimuth

# Test case: Make sure # points out = # points in
def test_num_points():
    in_points, num_points, cam_x, cam_y, altitude, pitch, azimuth = generate_args()

    out_points = get_hotspots_gps(in_points, cam_x, cam_y, altitude, pitch, azimuth)

    assert out_points.size == 2 * num_points, f"Wrong number of points outputted: {out_points.size}, should be {2 * num_points}"

# Test case: Check that the generated points are within camera view area for pitch = 0, azimuth = 0
def test_in_view():
    in_points, _, cam_x, cam_y, altitude, _, _ = generate_args()

    max_x = sensor_width * altitude / (2 * focal_length)
    max_y = sensor_height * altitude / (2 * focal_length)

    left, right = (cam_x - max_x*lat_factor), (cam_x + max_x*lat_factor)
    bottom, top = (cam_y - max_y*lat_factor), (cam_y + max_y*lat_factor)

    out_points = get_hotspots_gps(in_points, cam_x, cam_y, altitude, -1.57, 0)

    for point in out_points:
        print(f"in test_in_view: point = [{point[0][0], point[0][1]}], {left=}, {right=}, {bottom=}, {top=}")
        assert (left <= point[0][0] <= right) and (bottom <= point[0][1] <= top), \
            f"TEST FAILED: Point {point} is not in the expected range. {altitude=}, {cam_x=}, {cam_y=}, {max_x=}, {max_y=}."


# Test case: Check that the centre of the image has same coords as camera when camera pointed down
def test_image_centre():

    _, _, cam_x, cam_y, altitude, _, azimuth = generate_args()
    in_point = np.array([[1640., 1232.]])
    out_points = get_hotspots_gps(in_point, cam_x, cam_y, altitude, -1.57, azimuth)

    for point in out_points:
        print(f"in test_image_centre: point = [{point[0][0], point[0][1]}], {cam_x=}, {cam_y=}")
        assert ((point[0][0] - 0.000008) <= cam_x <= (point[0][0] + 0.000008)) and ((point[0][1] - 0.00001) <= cam_y <= (point[0][1] + 0.00001)), \
        f"TEST FAILED: point {point} at centre of image is not within about 1m of the camera coordinates"


def main():

    for func in (test_num_points, test_image_centre, test_in_view):
        try:
            func()
        except AssertionError as error:
            traceback.print_exc()


if __name__ == "__main__":
    main()