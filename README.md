# MQTT Radar-Camera Synchronization Stream

A project that communicates video frame data and radar processed data from external device clients to a server for IoT system synchronization on the end user device (like a Raspberry Pi) using MQTT. Essentially, device clients can publish externally to the broker, and the receiver can process it frame by frame as data is published by subscribing to the respective topics, allowing timestamps to be synchronized as well as radar data coordinates being synchronized with their corresponding video frames for visualization, detection and analysis purposes. The end goal is to integrate this with a fare gate set up in our laboratory to track evasions and other analytics.

## Installation

Install dependencies via pip:
```bash
pip install -r requirements.txt
```
Clone the repository:
```bash
git clone https://github.com/hsidd1/MQTT-RadCam-Stream.git
```
## Configuration

Configuration for this project's key parameters are in [config.yaml](config.yaml). 

### Parameters

`mqtt`: Config for MQTT processes

- `broker` (str) and `port` (int): broker address and port number to be connected to in [processModule/serverConnect.py](processModule/serverConnect.py)
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

## Usage

Execute main receiver to automatically subscribe to topics and run client programs as subprocesses in parallel
```bash
# publish and subscribe
python main.py
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
