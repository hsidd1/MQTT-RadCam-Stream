import numpy as np
import struct
import json
import datetime
import yaml

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

radar_data_file = config["Files"]["radar_data_file"]
with open(radar_data_file) as json_file:
    data = json.load(json_file)


# sensorhost format
def rd_process():
    radar_points = []
    for item in data["frames"]:
        num_ob = item["sensorMessage"]["metadata"]["numOfDetectedObjects"]
        detected_points = item["sensorMessage"]["object"]["detectedPoints"]
        timestamp = item["timestamp"]

        for j in range(num_ob):
            s = dict()
            s["sensorId"] = detected_points[j]["sensorId"]
            s["x"] = detected_points[j]["x"] * 10  # converting to mm
            s["y"] = detected_points[j]["y"] * 10
            s["z"] = detected_points[j]["z"] * 10
            s["timestamp"] = timestamp

            radar_points.append(s)
    return radar_points


data = rd_process()


def process_radar(radar_payload: list[dict]) -> None:
    # TODO: generate text and data points from radar data to plot over image
    raise NotImplementedError
