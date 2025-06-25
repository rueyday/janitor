import os
import PySimpleGUI as sg
import shutil

# Define paths to scan
PATHS_TO_SCAN = [
    os.path.expanduser("~\\AppData\\Local\\Temp"),
    os.path.expanduser("~\\Downloads"),
    "/tmp",
    "/var/tmp"
]

def scan_paths():
    suspicious_files = []
    for path in PATHS_TO_SCAN:
        if not os.path.exists(path):
            continue
        for root, _, files in os.walk(path):
            for file in files:
                full_path = os.path.join(root, file)
                if file.endswith(('.exe', '.bat', '.vbs')) or os.path.getsize(full_path) > 100*1024*1024:
                    suspicious_files.append(full_path)
    return suspicious_files

def delete_files(files):
    for file in files:
        try:
            os.remove(file)
        except Exception as e:
            print(f"Failed to delete {file}: {e}")

layout = [
    [sg.Text('Janitor Security Scanner')],
    [sg.Button('Scan'), sg.Button('Delete Selected'), sg.Button('Exit')],
    [sg.Listbox(values=[], size=(100, 20), select_mode=sg.SELECT_MODE_MULTIPLE, key='-RESULTS-')],
]

window = sg.Window('Janitor - Cleaner with Scanner', layout)

suspicious = []

while True:
    event, values = window.read()
    if event == sg.WINDOW_CLOSED or event == 'Exit':
        break
    elif event == 'Scan':
        suspicious = scan_paths()
        window['-RESULTS-'].update(suspicious)
    elif event == 'Delete Selected':
        to_delete = values['-RESULTS-']
        delete_files(to_delete)
        suspicious = [f for f in suspicious if f not in to_delete]
        window['-RESULTS-'].update(suspicious)

window.close()
