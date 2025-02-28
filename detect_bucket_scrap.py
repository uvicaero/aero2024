import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
from picamera2 import Picamera2

def detectBucket(videoLength):
    print("Detection Started")
    start = time.time()
    timePassed = 0

    listOfCenters = []

    # Initialize the Raspberry Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "YUV420", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Set up Matplotlib interactive mode
    plt.ion()
    fig, ax = plt.subplots()
    im_display = ax.imshow(np.zeros((480, 640), dtype=np.uint8), cmap="gray")  # Empty image placeholder
    plt.axis("off")  # Hide axes

    while timePassed < videoLength:
        # Capture grayscale image directly from PiCamera
        gray = picam2.capture_array("main")

        # Apply median blur
        gray = cv.medianBlur(gray, 5)

        # Detect circles using HoughCircles
        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT_ALT, dp=1.5, minDist=rows / 8,
                                  param1=300, param2=0.92,
                                  minRadius=0, maxRadius=0)

        # Convert grayscale to 3-channel image for drawing circles
        src_display = cv.cvtColor(gray, cv.COLOR_GRAY2BGR)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                print(f"Detected circle: Center={i[0], i[1]}, Radius={i[2]}")
                center = (i[0], i[1])
                listOfCenters.append(center)
                radius = i[2]
                cv.circle(src_display, center, 1, (0, 255, 0), 3)  # Circle center
                cv.circle(src_display, center, radius, (255, 0, 255), 3)  # Circle outline

        # Update Matplotlib display
        im_display.set_data(src_display)
        plt.pause(0.01)  # Allow time for the image to refresh

        # Update time
        timePassed = time.time() - start

    picam2.stop()
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show final frame

    return averageCenters(listOfCenters)

def averageCenters(centers):
    if not centers:
        return (None, None)  # Return None if no circles detected

    sumX = sum(c[0] for c in centers)
    sumY = sum(c[1] for c in centers)
    size = len(centers)

    print(f"Detected {size} circles over time.")

    return (int(sumX / size), int(sumY / size))

# ? **Main function to test `detectBucket(videoLength)`**
def main():
    print("\nRunning bucket detection test...\n")

    # Test for 5 seconds
    center = detectBucket(5)

    if center == (None, None):
        print("\nNo circles detected.")
    else:
        print(f"\nFinal averaged circle center: {center}")

if __name__ == "__main__":
    main()
