# UVic Aero 2024

# Boiler plate code for test output paths

from pathlib import Path

*Define the output directory relative to the root of the project*

project_root = Path(__file__).resolve().parent.parent
output_dir = project_root / 'data' / 'test_outputs' / '**Your_test_dir_here**'
output_dir.mkdir(parents=True, exist_ok=True)


*Import required functions from src/functions/your_function.py*

from src.functions.path_generator import generate_path, green_boundary_polygon, calculate_offset

*Define the output file path*

output_file = output_dir / 'tester_output.your_extension'

# Code for generating unique test output names
def get_unique_filename(output_dir, base_name="tester_output", extension="your extension here"):
    index = 1
    while True:
        filename = f"{base_name}_{index}{extension}"
        output_file = output_dir / filename
        if not output_file.exists():
            return output_file
        index += 1

