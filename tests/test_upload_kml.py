import pytest
import os
from upload_kml import upload_kml
from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from dotenv import load_dotenv

load_dotenv()

# Constants
folder_link1 = os.environ.get('TEST_FOLDER_1')
folder_link2 = os.environ.get('TEST_FOLDER_2')
kml1 = os.environ.get('TEST_FILE_1')
kml2 = os.environ.get('TEST_FILE_2')
kml3 = os.environ.get('TEST_FILE_3')
kml4 = os.environ.get('TEST_FILE_4')

# Helper function: returns user credentials
def get_credentials():
    # Defining Google Drive API scopes
    scopes = ['https://www.googleapis.com/auth/drive']

    # Loading service account key from .env file
    service_account_key = os.environ.get('SERVICE_ACCOUNT_KEY')

    # Use the service account details to create user credentials
    credentials = service_account.Credentials.from_service_account_info(info=service_account_key, scopes=scopes)
    return credentials


# Helper function: returns true if a file with the given name is found in the given folder
def is_file_in_folder(file, folder_link):
    # Seperating folder id from folder URL
    folder_id = (folder_link.split("/"))[-1]

    # Seperating the kml file's name from its filepath
    file_name = (file.split("/"))[-1]

    # Get user credentials
    credentials = get_credentials()

    return_bool = False
    try:
        # Create drive API client
        service = build('drive', 'v3', credentials=credentials)

        # Attempt to locate every file within the given folder
        files = (service.files().list(q="'" + folder_id + "' in parents")).execute()

        if files is not None:
            # For every file found
            for file in files.get("files", []):
                # If it is the requested file then return true
                if file.get("name") == file_name:
                    return_bool = True
                    break

    except HttpError as error:
        print(f"An error occurred locating file in drive:\n {error}")

    return return_bool


# Send files to a drive and verify they are there
def test_upload_kml():
    upload_kml(kml1, folder_link1)
    assert is_file_in_folder(kml1, folder_link1)

    upload_kml(kml2, folder_link1)
    assert is_file_in_folder(kml2, folder_link1)

    upload_kml(kml3, folder_link2)
    assert is_file_in_folder(kml3, folder_link2)

    upload_kml(kml4, folder_link2)
    assert is_file_in_folder(kml4, folder_link2)