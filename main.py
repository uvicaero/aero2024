#!/usr/bin/env python

from picamzero import Camera
import sys
sys.path.append("./utils")

from detect_hotspots import detect_hotspots
import time
from datetime import datetime

cam = Camera()
#cam.start_preview()

try:
    while True:
        timestamp = datetime.fromtimestamp(int(time.time())).strftime('%Y-%m-%d %H:%M:%S')
        path = f"./photos/photo_{timestamp}.jpg"
        cam.take_photo(path)
        print(f"\nPhoto taken: {path}")
        
        detect_hotspots(path, 0.9, debug=False)
        
        time.sleep(3)
except KeyboardInterrupt:
        #cam.stop_preview()
        print("Camera stopped")
