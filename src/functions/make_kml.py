#sample coordinates and description to test if the function is working

gps_coordinates = [
    (50.1023412, -110.7367821),
    (50.1027845, -110.7375123)
]
# formatting of list above causes parentheses to end up in kml file
# these are currently removed by replacing with nothing. 
# might be more elegant to change list format to avoid the parentheses in the first place?

source_descrip = "Some red balloons I guess?"
source_coordinates = 50.1023412, -110.7367821


def make_kml(coordinates, source_descrip, source_coordinates):
    #creating beginning of kml file
    kml_file = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
    <Document>
        <Placemark>
            <name>Source</name>
            <description>""" + source_descrip + """</description>
            <Point>\n\n
<coordinates>"""+ str(source_coordinates) +"""</coordinates>
            </Point>
        </Placemark>"""

    #inserting each hotspot into kml format
    hotspot_counter = 1
    for coordinate in coordinates:
        kml_file = kml_file + """        <Placemark>
            <name>Hotspot """ + (str(hotspot_counter)) + """</name>
            <Point>\n\n<coordinates>""" + str(coordinate) + """</coordinates>
            </Point>
        </Placemark>\n"""
        hotspot_counter = hotspot_counter + 1
    
    #closing kml file
    kml_file = kml_file +"""     </Document>
</kml>"""
    #removing parentheses that don't match kml format 
    kml_file = kml_file.replace("(", "")
    kml_file = kml_file.replace(")", "")

    #actually name and create the file
    file_path = "test_kml_file.kml"
    with open(file_path, 'w') as file:
        file.write (kml_file)
    # i don't know what the w argument above is for; got it here:  https://www.geeksforgeeks.org/create-a-new-text-file-in-python/
    return kml_file
    #maybe unnecessary to return kml_file since it makes an actual file, but nice to enable printing/check format


print make_kml(gps_coordinates, source_descrip, source_coordinates)


#is there a specific file path that should be specified, for compatibility with upload process?