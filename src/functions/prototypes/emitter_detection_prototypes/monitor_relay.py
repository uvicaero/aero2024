import RPi.GPIO as GPIO
import time

# Pin configuration
GPIO_PIN = 18

# Setup
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
GPIO.setup(GPIO_PIN, GPIO.IN)

try:
    print("Monitoring GPIO 23. Press Ctrl+C to exit.")
    while True:
        state = GPIO.input(GPIO_PIN)
        if state:
            print("GPIO 23 is ON")
        else:
            print("GPIO 23 is OFF")
        time.sleep(0.05)  # Adjust polling rate as needed

except KeyboardInterrupt:
    print("\nExiting program.")
    GPIO.cleanup()  # Reset GPIO settings
