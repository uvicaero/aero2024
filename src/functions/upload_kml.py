#! /usr/bin/env python3
import sys

from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from googleapiclient.errors import HttpError

def upload_kml(kml, drive_link):

    # Defining Google Drive API scopes and the location of the service account details
    scopes = ['https://www.googleapis.com/auth/drive']
    service_account_json_key = '/home/user/service_acc_key.json' # This is not a real key, need to find a secure way to access one

    # Seperating folder id from folder URL
    parent_folder_id = (drive_link.split("/"))[-1]
    # Seperating the kml file's name from its filepath
    file_name = (kml.split("/"))[-1]
    # Use the service account details to create user credentials
    credentials = service_account.Credentials.from_service_account_file(filename=service_account_json_key, scopes=scopes)

    file = None
    try:
        # Create drive API client
        service = build('drive', 'v3', credentials=credentials)

        # Specify file name and destination
        file_metadata = {
            'name': file_name,
            'parents': [parent_folder_id]
        }

        # Specify file type and contents
        media = MediaFileUpload(kml, mimetype="application/vnd.google-earth.kml+xml")

        # Create file in specified drive
        file = service.files().create(body=file_metadata, media_body=media, fields="id").execute()

    except HttpError as error:
        print(f"An error occurred uploading KML file to drive:\n {error}")

    if file != None:
        return file.get("id")
    else:
        return None