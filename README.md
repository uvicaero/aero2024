# UVic Aero 2024

# Boiler plate code for test output and import paths

import PATH
```
from pathlib import Path
```

Define the output directory relative to the root of the project
```
project_root = Path(__file__).resolve().parent.parent
output_dir = project_root / 'data' / 'test_outputs' / '**Your_test_dir_here**'
output_dir.mkdir(parents=True, exist_ok=True)
```

Import required functions from *src/functions/your_function.py*
```
from src.functions.path_generator import generate_path, green_boundary_polygon, calculate_offset
```
Define the output file path
```
output_file = output_dir / 'tester_output.your_extension'
```
# Code for generating unique test output names
```
def get_unique_filename(output_dir, base_name="tester_output", extension="your extension here"):
    index = 1
    while True:
        filename = f"{base_name}_{index}{extension}"
        output_file = output_dir / filename
        if not output_file.exists():
            return output_file
        index += 1
```

# Useful for testing: Cordinates of soft boundary for day 1 and how to make a polygon with them

Define the Soft Boundary polygon coordinates
```
green_boundary_coords = [
    (50.0971537, -110.7329257),
    (50.1060519, -110.7328869),
    (50.1060793, -110.7436756),
    (50.1035452, -110.7436555),
    (50.0989139, -110.7381534),
    (50.0971788, -110.7381487),
    (50.0971537, -110.7329257)
]
```

Create a Polygon for the green boundary
```
green_boundary_polygon = Polygon(green_boundary_coords)
```
