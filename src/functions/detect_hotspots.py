#!/usr/bin/env python

import argparse
import cv2
import numpy as np
from matplotlib import pyplot as plt

# numpy array should be grayscale
def detect_hotspots(image_or_path, threshold=0.7, debug=False):

    if isinstance(image_or_path, str):
        #load from file path
        image = cv2.imread(image_or_path)
        if image is None:
            print(f"Error: Unable to load image from {image_or_path}")
            return
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        #assume input is a NumPy array (image in memory)
        image = image_or_path

    thresh_val = round(255 * threshold)
    _, thresholded_image = cv2.threshold(image, thresh_val, 255, cv2.THRESH_BINARY)

    #if debug:
        #cv2.imshow("IR-image", image)
        #cv2.waitKey(0)
        #cv2.imshow("After Thresholding", thresholded_image)
        #cv2.waitKey(0)

    thresholded_image = thresholded_image.astype('uint8')

    #first param is img, second param is connectivity (4 or 8), last param is label depth
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresholded_image, 8, cv2.CV_32S)

    areas = stats[1:, cv2.CC_STAT_AREA] #first entry is background

    centroids = centroids[1:]
    
    filtered = centroids[areas >= 10] #keep components with area >= 100 pixels

    return filtered

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--image_or_path', type=str, required=True, help='Path to image to run inference on or numpy array in memory')
    parser.add_argument('--threshold', type=float, default=0.5, help='Threshold for binary threshold (between 0 and 1)')
    parser.add_argument('--debug', action='store_true', help='Shows images if set to true')
    
    args = parser.parse_args()

    res = detect_hotspots(**vars(args))
    print(res)

if __name__ == "__main__":
    main()
