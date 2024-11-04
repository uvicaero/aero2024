import cv2
import numpy as np
import math
import argparse


# points: list of uncorrected hotspot points (as tuples?)
# cam_x, cam_y: camera GPS coordinates
# pitch: EXIF tag GimbalPitch converted to radians, will always be negative
# azimuth: angle clockwise from N, in radians. EXIF tag FlightYawDegrees
# returns a list of tuples each containing the latitude and longitude of a hotspot
def get_image_points_gps(points, cam_x, cam_y, altitude, pitch, azimuth):

    # -> find. metres to latitude/longitude conversion factors for the current location (temp values)
    lat_factor = 1
    long_factor = 1

    # camera paramaters
    focal_length = 1 #  -> find. camera focal length in mm
    sensor_width = 1 # -> find. sensor width in mm
    aspect = 0.75 # aspect ratio, usually 3:4 or 9:16
    camera_matrix = 1 # -> find. comes from cam calibration
    dist_coeffs = 1 # -> find. comes from cam calibration
    img_half_height = 1 # -> find. number of pixels from centre of image to top of image
    image_half_width = 1 # -> find. 
    ratXh = sensor_width/focal_length/2 # ratio of sensor half-width to focal length (at image centre)
    ratYh = aspect * ratXh # ditto for sensor half-height
    phiYh = math.atan(ratYh) # 1/2-FOV angle in Y direction at image centre. Will be in radians.

    # still to come: applying undistortion to the points


    projected = [()]

    for point in points:

        frac_pixels_y = point[1] / img_half_height # calculate fraction of the point's pixels-to-centre / total pixels from top edge to centre
        frac_pixels_x = point[0] / image_half_width

        # k corresponds to the y-axis of the image, w to the x-axis of the image
        ground_k = altitude/math.tan(-pitch+frac_pixels_y*phiYh) # ground distance of camera ground projection to y-coordinate on image
        full_distance = math.sqrt(altitude^2+ground_k^2) # full distance, hypotenuse of ground distance and altitude triangle
        ground_w = full_distance * ratXh * frac_pixels_x # ground distance of camera ground position to x-coordinate on image

        # so now we have the ground distance from the camera to the point in meters (altitude units). and we need to add that distance to
        # the camera's current position. here is where we should check for which quadrant of image
        # the point is in, then make ground_k and ground_w positive or negative accordingly.

        # rotate using azimuth
        # x corresponds to latitude, y to longitude. need to convert cam GPS to meters for adding. i hope that works.
        world_x = cam_x / lat_factor + (ground_k) * math.cos(azimuth) + (ground_w) * math.sin(azimuth)
        world_y = cam_y / long_factor - (ground_k) * math.sin(azimuth) + (ground_w) * math.cos(azimuth)
        
        # convert from meters to lat/long degrees
        point_lat = lat_factor*world_x
        point_long = long_factor*world_y

        projected.append((point_lat, point_long))

    return projected

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument('--points', type=list, required=True, help='List of tuples representing hotspot points on the image')
    parser.add_argument('--cam_x', type=float, required=True, help='Latitude of camera')
    parser.add_argument('--cam_y', type=float, required=True, help='Longitude of camera')
    parser.add_argument('--altitude', type=float, required=True, help='Altitude of camera in meters')
    parser.add_argument('--pitch', type=float, default=0.0, help='Pitch of camera (radians from horizontal)')
    parser.add_argument('--azimuth', type=float, default=0.0, help='Azimuth of camera (radians clockwise from N)')
    
    args = parser.parse_args()

    result = get_image_points_gps(**vars(args))
    print(result)


if __name__ == "__main__":
    main()