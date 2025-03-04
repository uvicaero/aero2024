import sys
import cv2 as cv
import numpy as np
import time
import math


"""
This program uses the openCV detectBucket function to compute the displacement 
vector (difference between desired bucket location and detected bucket location)
"""
# Params: videoLength - Number of seconds camera runs for to get avg. position
# Return: (x_avg, y_avg, rad_avg) as a Tuple
def detectBucket(videoLength):
    print("Detection Started")
    start = time.time()
    timePassed = 0

    listOfCircles = []

    cam = cv.VideoCapture(0)
    
    while timePassed < videoLength:
        ret, src = cam.read()
        
        gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        
        gray = cv.medianBlur(gray, 5)

        rows = gray.shape[0]
        # HOUGH_GRADIENT_ALT is an variation of the algorithm which is sometimes more accurate
        # For this variation, param2 is the "accuracy threshold" with 1 being a perfect circle.
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT_ALT, dp=1.5, minDist=rows / 8,
                                param1=300, param2=0.92,
                                minRadius=0, maxRadius=0)
        

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                circle_info = (i[0], i[1], i[2])
                # circle center
                cv.circle(src, center, 1, (0, 100, 100), 3)
                listOfCircles.append(circle_info)
                # circle outline
                radius = i[2]
                cv.circle(src, center, radius, (255, 0, 255), 3)
        
        
        cv.imshow("detected circles", src)
        if cv.waitKey(1) == ord('q'):
            break

        end = time.time()
        timePassed = end - start

    return averageCenters(listOfCircles)

# Params: centers - List of all the circles detected and their info [x, y, rad]
# Return: (x_avg, y_avg, rad_avg) as a Tuple
def averageCenters(centers):
    if len(centers) < 1:
        return (-1, -1, -1) # error return value
    
    radius_values = []
    for c in centers:
        radius_values.append(c[2])

    rad_max = max(radius_values)
    rad_max_thresh = rad_max*0.9 # 90% of the max radius

    x_values = []
    y_values = []
    radius_values = []
    for c in centers:
        # Not factor in any circles less than the biggest rad
        if c[2] > rad_max_thresh:
            x_values.append(c[0])
            y_values.append(c[1])
            radius_values.append(c[2])
    print("Mean: ", rad_max)
    print("Mean Thresh: ", rad_max_thresh)

    # Possible idea to calculate average again and 
    # not factor in any values outside of the std dev

    return (int(np.mean(x_values)), int(np.mean(y_values)), int(np.mean(radius_values)))


# Computes the desired bucket location in pixels, adjusting for camera offset and altitude.
# Return: returns the desired location of the bucket as a tuple (x_desired, y_desired) in pixels.
def determine_desired_location(image_shape, camera_offset, bucket_real_radius, altitude, fov, detected_bucket_radius, scale_factor):
    """
    :param scale_factor: Dynamic scaling factor for adjusting size based on altitude. Default is 0.1. A larger scaling factor makes the adjustments more sensitive.
    """
    
    if len(fov) != 2:
        raise ValueError("FOV must be a tuple or list with two elements: (horizontal_fov, vertical_fov)")
    
    img_width, img_height = image_shape[:2]

    # Calculate the real-world dimensions of the camera's view at the current altitude
    real_width = 2 * altitude * math.tan(math.radians(fov[0] / 2))
    real_height = 2 * altitude * math.tan(math.radians(fov[1] / 2))

    # Compute pixels per meter
    pixels_per_meter_x = img_width / real_width 
    pixels_per_meter_y = img_height / real_height
    # Convert real-world bucket radius to expected pixels
    expected_bucket_radius_pixels_x = bucket_real_radius * pixels_per_meter_x
    expected_bucket_radius_pixels_y = bucket_real_radius * pixels_per_meter_y
   
    # Calculate the corrected image center based on camera offset
    x_center = int((img_width / 2) - (camera_offset[0]*pixels_per_meter_x))
    y_center = int((img_height / 2) - (camera_offset[1]*pixels_per_meter_y))

    # Calculate the size adjustment based on the detected radius (adjusts dynamically based on altitude)
    size_adjustment_x = (detected_bucket_radius - expected_bucket_radius_pixels_x) * (altitude * scale_factor)
    size_adjustment_y = (detected_bucket_radius - expected_bucket_radius_pixels_y) * (altitude * scale_factor)

    # Adjust the desired position uniformly
    x_desired = x_center + size_adjustment_x * 0.15 #0.15 is a scale factor to further adjust the desired position(It can be changed. A larger value can be used for a more sensitive adjustment of position)
    y_desired = y_center + size_adjustment_y * 0.15  
  

    return (int(x_desired), int(y_desired))

# Computes the displacement vector (difference between detected bucket location and expected bucket location)
# Return: returns the displacement vector as a tuple (displacement_x, displacement_y) in meters.
def displacement_in_meters(detected_location, image_shape, camera_offset, bucket_real_radius, altitude, fov, detected_bucket_radius, scale_factor):
  
    img_width, img_height = image_shape[:2]

    # Compute real-world dimensions of the camera's view at the current altitude
    real_width = 2 * altitude * math.tan(math.radians(fov[0] / 2))
    real_height = 2 * altitude * math.tan(math.radians(fov[1] / 2))

    # Compute conversion factors from pixels to meters
    meters_per_pixel_x = real_width / img_width
    meters_per_pixel_y = real_height / img_height

    # Use determine_desired_location to get the target position in pixels
    desired_location = determine_desired_location(image_shape, camera_offset, bucket_real_radius, altitude, fov, detected_bucket_radius, scale_factor)

    # Compute pixel displacement (might have to change the order of these while testing to see what works best (detected - desired or desired - detected))
    displacement_x_pixels = detected_location[0] - desired_location[0]
    displacement_y_pixels = desired_location[1] - detected_location[1]

    # Convert pixel displacement to meters
    displacement_x_meters = displacement_x_pixels * meters_per_pixel_x
    displacement_y_meters = displacement_y_pixels * meters_per_pixel_y

    # Debugging statements
    print(f"Real-world width: {real_width}m, height: {real_height}m")
    print(f"Meters per pixel (X): {meters_per_pixel_x}, Meters per pixel (Y): {meters_per_pixel_y}")
    print(f"Detected location (pixels): {detected_location}")
    print(f"Desired location (pixels): {desired_location}")
    print(f"Displacement (pixels): X={displacement_x_pixels}, Y={displacement_y_pixels}")
    print(f"Displacement (meters): X={displacement_x_meters}, Y={displacement_y_meters}")

    return (displacement_x_meters, displacement_y_meters)

# Params: circle_params - A tuple (x, y, rad) to print on the video window
# Return: void - This function simply displays a visual of the average circle data on the image
def displayAverage(circle_params):

    cam = cv.VideoCapture(0)
    ret, src = cam.read()
    center = (circle_params[0], circle_params[1])
    radius = circle_params[2]

    print("Center (pixels): ", center)
    print("Radius (pixels): ", radius)

    detected_bucket = circle_params

    #Values to change while testing:
    image_shape = (1280, 720)  # Camera resolution (width, height)
    camera_offset = (0, 0)  # Camera offset in meters (relative to drone's center)
    bucket_real_radius = 0.025  # Bucket radius in meters
    altitude = 0.70  # Drone altitude in meters
    fov = (66, 42)  # Camera FOV in degrees
    scale_factor = 0.1

    detected_bucket_radius = detected_bucket[2] # Detected bucket radius in pixels

    desired_location = determine_desired_location(image_shape, camera_offset, bucket_real_radius, altitude, fov, detected_bucket_radius, scale_factor)
    displacement_vector = displacement_in_meters(detected_bucket, image_shape, camera_offset, bucket_real_radius, altitude, fov, detected_bucket_radius, scale_factor)
   
    print("Dynamic desired bucket location in pixels: ", desired_location)
    print("Displacement vector in meters: ", displacement_vector)
    
    while True:
        # circle center
        cv.circle(src, center, 1, (0, 100, 100), 3)
        # circle outline
        cv.circle(src, center, radius, (0, 200, 150), 3)

        cv.imshow("average circle center", src)
        if cv.waitKey(1) == ord('q'):
            break

displayAverage(detectBucket(8))

