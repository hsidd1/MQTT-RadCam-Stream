import numpy as np
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


def load_data_tlv(data: json = None) -> list[dict]:
    radar_points = []
    if data is None:
        # add empty lists for everything
        return radar_points
    if isinstance(data, bytes):
        # decode JSON
        try:
            data = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
            return radar_points

    if isinstance(data, dict):
        # Handle the case when data is a single JSON object
        item = data
        x_list = item.get("x", [])
        y_list = item.get("y", [])
        z_list = item.get("z", [])
        for j in range(len(x_list)):
            s = dict()
            s["sensorId"] = item.get("Sensor_id", None)
            s["x"] = x_list[j] * 1000  # converting to mm
            s["y"] = y_list[j] * 1000
            s["z"] = z_list[j] * 1000
            time_str = item.get("time", "")
            if time_str:
                time_obj = datetime.datetime.strptime(time_str, "%H:%M:%S.%f")
                milliseconds = int(
                    time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second
                ) * 1000 + time_obj.microsecond // 1000
                s["timestamp"] = milliseconds
            radar_points.append(s)

    elif isinstance(data, list):
        # Handle the case when data is a list of JSON objects
        for item in data:
            item_dict = json.loads(item)
            x_list = item_dict.get("x", [])
            y_list = item_dict.get("y", [])
            z_list = item_dict.get("z", [])
            for j in range(len(x_list)):
                s = dict()
                s["sensorId"] = item_dict.get("Sensor_id", None)
                s["x"] = x_list[j] * 1000  # converting to mm
                s["y"] = y_list[j] * 1000
                s["z"] = z_list[j] * 1000
                time_str = item_dict.get("time", "")
                if time_str:
                    time_obj = datetime.datetime.strptime(time_str, "%H:%M:%S.%f")
                    milliseconds = int(
                        time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second
                    ) * 1000 + time_obj.microsecond // 1000
                    s["timestamp"] = milliseconds
                radar_points.append(s)

    return radar_points 
