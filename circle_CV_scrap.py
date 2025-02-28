import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from picamera2 import Picamera2

# Initialize the Raspberry Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

plt.ion()  # Enable interactive mode

fig, ax = plt.subplots()
im_display = ax.imshow(np.zeros((480, 640), dtype=np.uint8), cmap="gray")  # Grayscale placeholder
plt.axis("off")

try:
    while True:
        # Capture frame (in RGB format)
        src = picam2.capture_array("main")  
        gray = cv.cvtColor(src, cv.COLOR_RGB2GRAY)  # Convert to grayscale

        # Apply median blur
        gray = cv.medianBlur(gray, 5)

        # Detect circles using HoughCircles
        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT_ALT, dp=1.5, minDist=rows / 8,
                                   param1=300, param2=0.92,
                                   minRadius=0, maxRadius=0)

        # Convert grayscale to 3-channel image for drawing circles
        src_display = cv.cvtColor(gray, cv.COLOR_GRAY2RGB)

        # Draw detected circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                print(i)  # Print circle details
                center = (i[0], i[1])
                cv.circle(src_display, center, 1, (0, 255, 0), 3)  # Circle center
                radius = i[2]
                cv.circle(src_display, center, radius, (255, 0, 255), 3)  # Circle outline

        # Update Matplotlib display
        im_display.set_data(src_display)
        plt.pause(0.01)

except KeyboardInterrupt:
    print("\nStopped by user.")

# Stop camera
picam2.stop()
plt.ioff()
plt.show()
