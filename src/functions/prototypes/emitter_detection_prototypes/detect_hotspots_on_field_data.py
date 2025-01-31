import os
import cv2
import numpy as np
from src.functions.detect_hotspots import detect_hotspots
from src.functions.prototypes.emitter_detection_prototypes.detect_hotspots_with_mask import detect_hotspots_with_mask

def process_images(directory, threshold=0.7):
    results = []
    output_dir = os.path.join(directory, "output")
    os.makedirs(output_dir, exist_ok=True)
    
    if not os.path.exists(directory):
        print(f"Error: Directory {directory} does not exist.")
        return
    
    for filename in os.listdir(directory):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')):
            filepath = os.path.join(directory, filename)
            image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
            
            if image is None:
                print(f"Error: Unable to load image {filename}")
                continue
            
            hotspots = detect_hotspots_with_mask(image, threshold=threshold)
            num_hotspots = len(hotspots) if hotspots is not None else 0
            results.append((filename, num_hotspots, hotspots))
    
    for filename, num_hotspots, hotspots in results:
        if num_hotspots == 1:
            print(f"{filename}: 1 hotspot detected at location {hotspots[0]}")
        else:
            print(f"{filename}: {num_hotspots} hotspots detected")
    
    return results

if __name__ == "__main__":
    directory = "data/field_data/ir_detection_test_images"
    process_images(directory)
