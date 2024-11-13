import os
def generate_kml(hotspot_coords): # hotspot_coords is a list of tuples 
    # Start the KML document with the necessary XML headers and opening <Document> tag
    kml_content = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
    <Document>
'''

    # Loop through the list of hotspot coordinates and create a Placemark for each
    for i, (lat, lon) in enumerate(hotspot_coords, start=1):
        kml_content += f'''
        <Placemark>
            <name>Hotspot {i}</name>
            <Point>
                <coordinates>{lon},{lat},0</coordinates>
            </Point>
        </Placemark>
'''

    # Close the <Document> and </kml> tags
    kml_content += '''
    </Document>
</kml>
'''

    # Save the KML content to a .kml file
    with open("hotspots.kml", "w") as kml_file:
        kml_file.write(kml_content)# Import the function from make_kml.py

def setup_module(module):
    """Setup before any test runs (clears existing KML file)."""
    if os.path.exists("hotspots.kml"):
        os.remove("hotspots.kml")

def test_generate_kml_basic():
    """Test the function with a simple list of coordinates."""
    hotspot_coords = [(50.1023412, -110.7367821), (50.1027845, -110.7375123)]
    generate_kml(hotspot_coords)
    
    # Check if the KML file was created
    assert os.path.exists("hotspots.kml")
    
    with open("hotspots.kml", "r") as kml_file:
        kml_content = kml_file.read()
        assert "<coordinates>-110.7367821,50.1023412,0</coordinates>" in kml_content
        assert "<coordinates>-110.7375123,50.1027845,0</coordinates>" in kml_content
        assert "<name>Hotspot 1</name>" in kml_content
        assert "<name>Hotspot 2</name>" in kml_content

def test_empty_coordinates():
    """Test the function with an empty list of coordinates."""
    hotspot_coords = []
    generate_kml(hotspot_coords)

    assert os.path.exists("hotspots.kml")

    with open("hotspots.kml", "r") as kml_file:
        kml_content = kml_file.read()
        assert "<Placemark>" not in kml_content

def test_single_coordinate():
    """Test the function with a single coordinate."""
    hotspot_coords = [(50.1023412, -110.7367821)]
    generate_kml(hotspot_coords)

    assert os.path.exists("hotspots.kml")

    with open("hotspots.kml", "r") as kml_file:
        kml_content = kml_file.read()
        assert "<name>Hotspot 1</name>" in kml_content
        assert "<coordinates>-110.7367821,50.1023412,0</coordinates>" in kml_content

import os
print(os.getcwd())  # Print the current working directory
