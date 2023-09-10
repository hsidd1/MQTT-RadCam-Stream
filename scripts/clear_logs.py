"""
A script to easily delete all logs quickly instead of manually deleting them.
"""
import os

RADCAMLOG = "../liveDataLog/radcam_log.json"
IMGS = "../liveDataLog/camera_data/"

with open (RADCAMLOG, "w") as f:
    f.write("")
    print("Cleared radcam_log.json")

# delete all jpgs
for file in os.listdir(IMGS):
    if file.endswith(".jpg"):
        os.remove(IMGS + file)
        #print(f"Deleted {file}")
print("Cleared camera_data folder.")