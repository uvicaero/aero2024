import cv2
from picamera2 import Picamera2

def main():
        # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (3280, 2464)}  # Maximum resolution
    )
    picam2.configure(config)
    picam2.start()


    print("\nCapturing image...")
    rgb_image = picam2.capture_array("main")
    image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    cv2.imshow("IR-image", image)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()