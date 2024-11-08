# Useful things

## Using Raspberry pi RAM with camera

Directory to save to RAM
```
rpicam-still --shutter 1000 -o /dev/shm/image.jpg
```
Periodically save to RAM
```
timestamp=$(date +%s)
rpicam-still --shutter 1000 -o /dev/shm/image_$timestamp.jpg
```
Clear images from RAM
```
rm /dev/shm/*.jpg
```
Loading from RAM
```
# Load the image directly from RAM
image = cv2.imread('/dev/shm/image.jpg')

# Process the image (e.g., hotspot detection)
result = process_image(image)
```

## rpicam-still flags

```
# Capture a photo with the following options:
# --shutter: Sets the shutter speed to 1/1000s (1000 microseconds). A fast shutter speed reduces motion blur.
# --gain: Controls the analog gain (similar to ISO). Increasing gain compensates for faster shutter speeds by amplifying the sensor's signal.
# --brightness: Adjusts the brightness level of the image (range: 0 to 100). Helps fine-tune the exposure if needed.
# -o: Specifies the output file path. Here, we're saving to RAM in /dev/shm for faster access.

rpicam-still --shutter 1000 --gain 8 --brightness 50 -o /dev/shm/image.jpg
```

1000 = 1/1000s (1 ms).
5000 = 1/200s (5 ms).
10000 = 1/100s (10 ms).
