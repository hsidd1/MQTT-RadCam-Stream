#!/bin/bash

RADCAMLOG="liveDataLog/radcam_log.json"
IMGS="liveDataLog/camera_data/"

# clear radcam_log.json
> "$RADCAMLOG"
echo "Cleared radcam_log.json"

# delete all jpgs
for file in "$IMGS"*.jpg; do
    rm "$file"
    #echo "Deleted $file"
done
echo "Cleared camera_data folder."