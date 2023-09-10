# MQTT Radar-Camera Synchronization Stream

A project that communicates video frames and radar data from external device clients to a server for IoT system synchronization on the end user device (like a Raspberry Pi) using MQTT. Essentially, device clients can publish externally to the broker, and the receiver can process it frame by frame as data is published by subscribing to the respective topics, allowing timestamps to be synchronized as well as radar data coordinates being synchronized with their corresponding video frames for visualization, detection and analysis purposes. Frames are displayed in real time with two radars detecting objects within that frame, which are then plotted on the frame for the user. This software is being used in a laboratory setting currently for visualizing traffic, detecting evasions and tracking analytics in a fare gate system, hence the camera and radar positioning relative to each other is specific for this setup. I've also added pre-recorded video footage and radar data that are processed frame by frame for demonstration and validation which can be found in this repository as well. Feel free to try it out yourself.

## Installation

Clone the repository:
```bash
git clone https://github.com/hsidd1/MQTT-RadCam-Stream.git
```
Install dependencies via pip:
```bash
pip install -r requirements.txt
```
## Configuration - General

Configuration for this project's key parameters are in [config.yaml](config.yaml). 

### Parameters

`mqtt`: Config for MQTT processes

- `broker` (str) and `port` (int): broker address and port number to be connected to in [processModule/serverConnect.py](processModule/serverConnect.py)
 m,,,,,,,,n 

### Local Host Broker Setup

This program requires a very rapid subscribe and publish rate which all tested web-based brokers could not accomplish. The workaround here is running on localhost server. 

- Install Eclipse Mosquitto: https://mosquitto.org/download/
- Open terminal as administrator:
```bash
cd <path-to-mosquitto>
net start mosquitto # run broker
```
- Ensure config for `broker` is set to localhost: `"127.0.0.1"`

## Live Data
#### Configuration
Radar used is IWR6843 by TI (two of them). Make sure these are connected and setup with the necessary drivers. The radar config file is included within the repository. 

Data saving is done in `live_receiver.py` in a thread. See details on format in [processModule/save_data.py](processModule/save_data.py). Comment out the thread `t.start()` for when you do not want to update logs.

In [config.yaml](config.yaml) under `LiveData`, update `DataPort` and `CLIport` parameters matching devices.

Disconnect and reconnect radar after each session to avoid blank data transfer.

Camera used is Logitech BRIO (any USB camera works). Ensure external camera is connected to device.

### Usage for Live Data
Run the live receiver for live data, which runs live camera and live radar as subprocesses:
```bash
python live_receiver.py
```
These individual clients can be run individually as well:
```bash
# Optional
python live_cameraclient.py
python live_radarclient.py
```
## Pre-Processed Data 

### Configuration - Pre-Processed Data

- `show_log`: (bool) If `True`, attach `on_message` property to clients for logging. Provides additional information on publishing (QoS, retain, buf).
- `compressed_output`: (bool) If `True`, prints shortened version of published data to console. Frame byte arrays are massive, and often this whole array will clutter console and slow down processes when not needed.

`Files`: Change files to be processed 
- `video_file`: (str) .avi or other supported video format by cv2
-  `radar_data_file`: (str) .json in same format as [data/radardata.json](data/radardata.json) for processing in [processModule/rd_process.py](processModule/rd_process.py) to work.
-  Note: If changing video file, then run script [scripts/image_extract.py](scripts/image_extract.py) with the relevant paths. This is for subscriber processing and is explained in more detail in the script comments.
  
`CameraOutput`: Config for visualization of frames received from camera

- `fps`: (int) Number of frames to be transmitted per second.
- `frame_start`: (int) Frame to begin transfer at. First few frames do not process. Use to jump to specific frame id.
- `frame_skip`: (int) Send every nth frame. Ex. `frame_skip = 30` sends every 30th frame.
- `continuous_frame_mode`: (bool) If `True`, Displays incoming frames without waiting for user to close window (simulate video playback).

`write_log`: (bool) If `True`, write receiving logs to a log file.

### Publishing, Receiving and Processing 

Execute main receiver to automatically subscribe to topics and run client programs as subprocesses in parallel
```bash
# publish and subscribe
python main.py
#Optional additional flag for logging:
python main.py <log-file-path> # change log file path
```
To run receiver client only without other clients activating:
```bash
python receiver_client.py
```
To run publishing clients individually:
```bash
# run camera client
python cam_client.py
# run radar client
python radar_client.py
```
Disconnect broker when completed:
```bash
# open as administrator
cd <path-to-mosquitto>
net stop mosquitto
```