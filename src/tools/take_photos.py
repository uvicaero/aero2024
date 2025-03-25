import os
import sys
import subprocess
from picamera2 import Picamera2
import time

# Check for correct arguments
if len(sys.argv) != 2 or sys.argv[1] not in ['noir', 'wide']:
    print("Usage: python3 script.py [noir|wide]")
    sys.exit(1)

# Set resolution and directory based on flag
camera_type = sys.argv[1]

if camera_type == 'noir':
    RESOLUTION = (3280, 2464)
    IMAGE_DIR = 'data/calibration_images/noir'
elif camera_type == 'wide':
    RESOLUTION = (1280, 720)
    IMAGE_DIR = 'data/calibration_images/wide'


# Function to generate an unused filename
def get_unused_filename(directory, prefix='image', ext='jpg'):
    index = 1
    while True:
        filename = f"{prefix}_{index:03d}.{ext}"
        full_path = os.path.join(directory, filename)
        if not os.path.exists(full_path):
            return full_path
        index += 1

# Initialize camera
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(main={"size": RESOLUTION})
picam2.configure(camera_config)
picam2.start()

# Warm-up time
print(f"Warming up {camera_type} camera at resolution {RESOLUTION}...")
time.sleep(1)

print("Press Enter to capture a photo. Type 'q' then Enter to quit.")

try:
    while True:
        user_input = input("Capture photo? ")
        if user_input.lower() == 'q':
            print("Exiting...")
            break

        temp_filename = "/tmp/temp_capture.jpg"
        picam2.capture_file(temp_filename)
        time.sleep(0.2)
        subprocess.Popen(['xdg-open', temp_filename])

        save_input = input("Save this photo? (y/n): ")
        if save_input.lower() == 'y':
            filename = get_unused_filename(IMAGE_DIR)
            os.rename(temp_filename, filename)
            print(f"Photo saved as: {filename}")
        else:
            os.remove(temp_filename)
            print("Photo discarded.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    picam2.stop()
