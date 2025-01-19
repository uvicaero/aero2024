import os
from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from googleapiclient.errors import HttpError
from dotenv import load_dotenv

# Uploads a given KML file to a given folder, assuming the service account associated with SERVICE_ACCOUNT_KEY in the .env file has been given access to said folder.

# The folder ID is pulled off the end of the given link.
# The ID can also be passed into the function in place of the link.
def upload_kml(kml, folder_link):

    # Defining Google Drive API scopes
    scopes = ['https://www.googleapis.com/auth/drive']

    # Loading service account key from .env file
    load_dotenv()
    service_account_key = os.environ.get('SERVICE_ACCOUNT_KEY')

    # Seperating folder id from folder URL
    folder_id = (folder_link.split("/"))[-1]

    # Seperating the kml file's name from its filepath
    file_name = (kml.split("/"))[-1]

    # Use the service account details to create user credentials
    credentials = service_account.Credentials.from_service_account_info(info=service_account_key, scopes=scopes)

    file = None
    try:
        # Create drive API client
        service = build('drive', 'v3', credentials=credentials)

        # Specify file name and destination
        file_metadata = {
            'name': file_name,
            'parents': [folder_id]
        }

        # Specify file type and contents
        media = MediaFileUpload(kml, mimetype="application/vnd.google-earth.kml+xml")

        # Create file in specified drive
        file = service.files().create(body=file_metadata, media_body=media, fields="id").execute()

    except HttpError as error:
        print(f"An error occurred uploading KML file to drive:\n {error}")

    if file is not None:
        return file.get("id")
    else:
        return None