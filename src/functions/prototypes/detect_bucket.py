import sys
import cv2 as cv
import numpy as np
import time

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

# Params: circle_params - A tuple (x, y, rad) to print on the video window
# Return: void - This function simply displays a visual of the average circle data on the image
def displayAverage(circle_params):
    cam = cv.VideoCapture(0)
    ret, src = cam.read()
    center = (circle_params[0], circle_params[1])
    radius = circle_params[2]

    print("Center: ", center)
    print("Radius: ", radius)

    while True:
        # circle center
        cv.circle(src, center, 1, (0, 100, 100), 3)
        # circle outline
        cv.circle(src, center, radius, (0, 200, 150), 3)

        cv.imshow("average circle center", src)
        if cv.waitKey(1) == ord('q'):
            break


# Circle Info: (0, 0) is top left of frame
displayAverage(detectBucket(3))

