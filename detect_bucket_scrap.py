import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
from picamera2 import Picamera2

def detectBucket(videoLength):
    print("Detection Started")
    start = time.time()
    timePassed = 0

    listOfCircles = []

    # Initialize the Raspberry Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "YUV420", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Video writer setup
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    video_filename = "temp_video.avi"
    out = cv.VideoWriter(video_filename, fourcc, 20.0, (640, 480))

    while timePassed < videoLength:
        # Capture grayscale image directly from PiCamera
        gray = picam2.capture_array("main")
        gray = cv.cvtColor(gray, cv.COLOR_YUV2GRAY_YV12)

        # Apply median blur
        gray = cv.medianBlur(gray, 5)

        # Detect circles using HoughCircles
        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT_ALT, dp=1.5, minDist=rows / 8,
                                  param1=300, param2=0.97,
                                  minRadius=0, maxRadius=0)

        # Convert grayscale to 3-channel image for drawing circles
        src_display = cv.cvtColor(gray, cv.COLOR_GRAY2BGR)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                print(f"Detected circle: Center={i[0], i[1]}, Radius={i[2]}")
                center = (i[0], i[1])
                circle_info = (i[0], i[1], i[2])
                listOfCircles.append(circle_info)
                radius = i[2]
                cv.circle(src_display, center, 1, (0, 255, 0), 3)  # Circle center
                cv.circle(src_display, center, radius, (255, 0, 255), 3)  # Circle outline

        # Write frame to video
        out.write(src_display)

        # Update time
        timePassed = time.time() - start

    picam2.stop()
    out.release()

    # Play the saved video using Matplotlib
    playVideo(video_filename)

    return averageCenters(listOfCircles)

def playVideo(video_filename):
    cap = cv.VideoCapture(video_filename)
    plt.ion()
    fig, ax = plt.subplots()
    im_display = ax.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
    plt.axis("off")
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        im_display.set_data(frame)
        plt.pause(0.05)
    
    cap.release()
    plt.ioff()
    plt.show()

def averageCenters(centers):
    if len(centers) < 1:
        return (-1, -1, -1)  # error return value
    
    radius_values = [c[2] for c in centers]
    rad_max = max(radius_values)
    rad_max_thresh = rad_max * 0.9  # 90% of the max radius

    x_values, y_values, radius_values = [], [], []
    for c in centers:
        if c[2] > rad_max_thresh:
            x_values.append(c[0])
            y_values.append(c[1])
            radius_values.append(c[2])

    print(f"Mean Radius: {rad_max}")
    print(f"Mean Threshold: {rad_max_thresh}")

    return (int(np.mean(x_values)), int(np.mean(y_values)), int(np.mean(radius_values)))

def displayAverage(circle_params):
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "YUV420", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    
    src = picam2.capture_array("main")
    src = cv.cvtColor(src, cv.COLOR_YUV2BGR_YV12)

    center = (circle_params[0], circle_params[1])
    radius = circle_params[2]

    print("Final Center: ", center)
    print("Final Radius: ", radius)

    # Draw circle on image
    cv.circle(src, center, 1, (0, 100, 100), 3)  # Circle center
    cv.circle(src, center, radius, (0, 200, 150), 3)  # Circle outline
    
    plt.imshow(src)
    plt.axis("off")
    plt.show()

    picam2.stop()

def main():
    print("\nRunning bucket detection test...\n")
    
    center = detectBucket(5)
    if center == (-1, -1, -1):
        print("\nNo circles detected.")
    else:
        print(f"\nFinal averaged circle data: Center={center[:2]}, Radius={center[2]}")
        displayAverage(center)

if __name__ == "__main__":
    main()
