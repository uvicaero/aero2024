import cv2 as cv
import numpy as np
import math
import argparse

# constants
lat_factor = 1/111230.9531021928 # 1 degree of latitude per that many meters, at 50.100 degrees latitude
long_factor = 1/71546.90282746412 # 1 degree of longitude per that many meters, at 50.100 degrees latitude

sensor_width = 3.674 # sensor width in mm 
sensor_height = 2.760 # sensor height in mm
camera_matrix = np.array([[2398.54998, 0., 1696.02043],
                            [0., 2406.88752, 1322.32359],
                            [0., 0., 1.]]) # from cam calibration
dist_coeffs = np.array([0.220794232, -0.522929233, -0.000112948804, 0.00213028546, 0.292007192]) # from cam calibration
focal_length = 3.04 #in mm. camera_matrix[0][0] camera focal length in units from calibration. USUALLY. but calibration needs work
img_half_height = 3280 / 2 # number of pixels from centre of image to top of image
img_half_width = 2464 / 2 # number of pixels from centre to side of image
rat_x = (sensor_width/focal_length)/2 # ratio of sensor half-width to focal length (at image centre)
rat_y = (sensor_height/focal_length)/2 # ditto for sensor half-height
phi_y = math.atan(rat_y) # 1/2-FOV angle in Y direction at image centre. Will be in radians.
# note for above: could probably find FOV directly instead


# distorted_points: hotspot image points as 2xN numpy array OR comma-separated string of alternating x, y values
# cam_x, cam_y: camera GPS coordinates
# pitch: EXIF tag GimbalPitch converted to radians, will always be negative
# azimuth: angle clockwise from north, in radians. EXIF tag FlightYawDegrees
# returns an array containing the latitude and longitude of each hotspot
def get_hotspots_gps(distorted_points, cam_x, cam_y, altitude, pitch, azimuth):

    # undistort the points
    if isinstance(distorted_points, str):
        dist_pts = np.fromstring(distorted_points, sep=", ").reshape((-1, 2))
    else:
        dist_pts = distorted_points

    points = cv.undistortImagePoints(dist_pts, camera_matrix, dist_coeffs)
    projected = np.zeros(points.shape, np.float32)

    i = 0
    for point in points:

        frac_pixels_y = (point[0][1] - img_half_height) / img_half_height # calculate fraction of the point's pixels-to-centre / total pixels from top edge to centre
        frac_pixels_x = (point[0][0] - img_half_width) / img_half_width

        # k corresponds to the y-axis direction from the image, w to the x-axis direction from the image
        ground_k = altitude/math.tan(-1*pitch+frac_pixels_y*phi_y) # ground distance of camera ground projection to y-coordinate on image
        full_distance = math.sqrt(altitude*altitude+ground_k*ground_k) # full distance, hypotenuse of ground distance and altitude triangle
        ground_w = full_distance * rat_x * frac_pixels_x # ground distance of camera ground position to x-coordinate on image

        # rotate using azimuth
        # x corresponds to latitude, y to longitude. need to convert cam GPS to meters for adding.
        # add/subtract may need to be tweaked once we can visualise with real-world data
        world_x = cam_x / lat_factor + (ground_k) * math.cos(azimuth) + (ground_w) * math.sin(azimuth)
        world_y = cam_y / long_factor - (ground_k) * math.sin(azimuth) + (ground_w) * math.cos(azimuth)
        
        # convert from meters to lat/long degrees
        point_lat = lat_factor*world_x
        point_long = long_factor*world_y

        projected[i] = [point_lat, point_long]
        i += 1

    return projected

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument('--distorted_points', type=str, required=True, help='1D string with list of image points eg "x1, y1, x2, y2"')
    parser.add_argument('--cam_x', type=float, required=True, help='Latitude of camera')
    parser.add_argument('--cam_y', type=float, required=True, help='Longitude of camera')
    parser.add_argument('--altitude', type=float, required=True, help='Altitude of camera in meters')
    parser.add_argument('--pitch', type=float, default=0.0, help='Pitch of camera (radians from horizontal)')
    parser.add_argument('--azimuth', type=float, default=0.0, help='Azimuth of camera (radians clockwise from N)')
    
    args = parser.parse_args()

    result = get_hotspots_gps(**vars(args))
    print(result)


if __name__ == "__main__":
    main()