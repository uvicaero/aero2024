import sys
import cv2 as cv
import numpy as np
import time

def detectBucket(videoLength):
    print("Detection Started")
    start = time.time()
    timePassed = 0
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
                print(i)
                center = (i[0], i[1])
                # circle center
                cv.circle(src, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv.circle(src, center, radius, (255, 0, 255), 3)
        
        
        cv.imshow("detected circles", src)
        if cv.waitKey(1) == ord('q'):
            break


        end = time.time()
        timePassed = end - start


detectBucket(3)

