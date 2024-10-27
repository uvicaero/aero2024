#!/usr/bin/env python

import argparse
import cv2
import numpy as np
from matplotlib import pyplot as plt

def detect_hotspots(img_path, threshold=0.5, debug=False):
    image = cv2.imread(img_path)

    #check if image is loaded correctly
    if image is None:
        print(f"Error: Unable to load image from {img_path}")
        return

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    thresh_val = round(255 * threshold)
    _, thresholded_image = cv2.threshold(gray_image, thresh_val, 255, cv2.THRESH_BINARY)

    if debug:
        cv2.imshow("IR-image", gray_image)
        k = cv2.waitKey(0)
        cv2.imshow("After Thresholding", thresholded_image)
        k = cv2.waitKey(0)

    thresholded_image = thresholded_image.astype('uint8')

    #first param is img, second param is connectivity (4 or 8), last param is label depth
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresholded_image, 8, cv2.CV_32S)

    areas = stats[1:, cv2.CC_STAT_AREA] #first entry is background

    print(centroids)
    centroids = centroids[1:]

    filtered = centroids[areas >= 100] #keep components with area >= 100 pixels

    return filtered

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--img_path', type=str, required=True, help='Path to image to run inference on')
    parser.add_argument('--threshold', type=float, default=0.5, help='Threshold for binary threshold (between 0 and 1)')
    parser.add_argument('--debug', action='store_true', help='Shows images if set to true')
    
    args = parser.parse_args()

    res = detect_hotspots(**vars(args))
    print(res)
