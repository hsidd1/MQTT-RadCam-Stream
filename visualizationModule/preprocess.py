import numpy as np
from .radar_points import RadarData
import json
import datetime

# for entry sensor
def calc_rot_matrix(alpha, beta):
    """alpha is the angle along z axis - yaw
    beta is the angle along x axis - pitch
    gamma is the angle along y axis - roll, not used here
    all angles are in degrees and counter.
    Rototation matrix is calculated in the order of z -> x -> y
    """
    rotz = np.zeros((3, 3))
    rotz[0, 0] = np.cos(np.radians(alpha))
    rotz[0, 1] = -np.sin(np.radians(alpha))
    rotz[1, 0] = np.sin(np.radians(alpha))
    rotz[1, 1] = np.cos(np.radians(alpha))
    rotz[2, 2] = 1
    rotx = np.zeros((3, 3))
    rotx[0, 0] = 1
    rotx[1, 1] = np.cos(np.radians(beta))
    rotx[1, 2] = -np.sin(np.radians(beta))
    rotx[2, 1] = np.sin(np.radians(beta))
    rotx[2, 2] = np.cos(np.radians(beta))
    return rotz, rotx


def rot_mtx_entry(alpha, beta):
    return calc_rot_matrix(alpha, beta)


def rot_mtx_exit(alpha, beta):
    return calc_rot_matrix(alpha + 180, beta)


def load_data_sensorhost(data: json) -> RadarData:
    radar_points = []
    for item in data["frames"]:
        num_ob = item["sensorMessage"]["metadata"]["numOfDetectedObjects"]
        detected_points = item["sensorMessage"]["object"]["detectedPoints"]
        timestamp = item["timestamp"]  # world time?

        for j in range(num_ob):
            s = dict()
            s["sensorId"] = detected_points[j]["sensorId"]
            s["x"] = detected_points[j]["x"] * 10  # converting to mm
            s["y"] = detected_points[j]["y"] * 10
            s["z"] = detected_points[j]["z"] * 10
            s["timestamp"] = timestamp

            radar_points.append(s)
    return RadarData(radar_points)

def load_data_tlv(data: json = None) -> RadarData:
    if data is None:
        # add empty lists for everything
        return RadarData([])
    radar_points = []
    for item in data:
        for j in range(len(item["x"])):
            s = dict()
            s["sensorId"] = item["Sensor_id"]
            s["x"] = item["x"][j] * 1000  # converting to mm
            s["y"] = item["y"][j] * 1000
            s["z"] = item["z"][j] * 1000
            #s["timestamp"] = int(item["time"].replace(":","").replace(".",""))
            time_str = item["time"]
            time_obj = datetime.datetime.strptime(time_str, "%H:%M:%S.%f")

            # convert datetime to milliseconds
            milliseconds = int(time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second) * 1000 + time_obj.microsecond // 1000
            s["timestamp"] = milliseconds
            radar_points.append(s)

    return RadarData(radar_points)