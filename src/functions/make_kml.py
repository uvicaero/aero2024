# For this task write a python function that takes a list of GPS coordinates and output a KML with the GPS coordinates.
#The function should be called with a list of GPS coordinates 

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

    # Define the file path
    kml_path = "hotspots.kml"

    # Save the KML content to the specified file path
    with open(kml_path, "w") as kml_file:
        kml_file.write(kml_content)

    return kml_path

