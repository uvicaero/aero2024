#!/usr/bin/env python

from picamzero import Camera
import sys
sys.path.append("./utils")

import detect_hotspots
import time

cam = Camera()
#cam.start_preview()

try:
    while True:
        timestamp = int(time.time())
        path = f"./photos/photo_{timestamp}.jpg"
        cam.take_photo(path)
        print(f"\nPhoto taken: {path}")
        
        detect_hotspots(path, 0.9, debug=True)
        
        time.sleep(3)
except KeyboardInterrupt:
        #cam.stop_preview()
        print("Camera stopped")
