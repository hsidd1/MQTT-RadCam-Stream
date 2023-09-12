import numpy as np


class RadarFrame:
    def __init__(self, data: "list[dict[str, int or float]]"):
        """
        Radar frame object contains data for a defined frame interval in lists for each attribute
        param data: a list of dictionaries
        ex. [{
                'sensorId': 2,
                'x': -280.35359191052436,
                'y': 524.516705459526,
                'z': 875.3924645059872,
                'timestamp': 1663959822542,
                'isStatic: 0
            }, ...]
        """
        self.sid = []
        self.x = []
        self.y = []
        self.z = []
        self.ts = []
        self.is_static = []  # -1 default, 1 static, 0 not static.
        for item in data:
            self.sid.append(item["sensorId"])
            self.x.append(item["x"])
            self.y.append(item["y"])
            self.z.append(item["z"])
            self.ts.append(item["timestamp"])
            self.is_static.append(-1)  # update in main program with static points class

    def __repr__(self):
        class_str = f"RadarFrame object with {len(self.sid)} points."
        if len(self.sid) > 0:
            class_str += f" Sensor id: {set(self.sid)}, starting ts: {self.ts[0]}, ending ts: {self.ts[-1]}"
        return class_str

    # check if a specified sensor is empty
    def is_empty(self, target_sensor_id=None) -> bool:
        # if sensor id is not passed in, check all sensors
        if target_sensor_id is None:
            return len(self.sid) == 0
        else:
            # if argument specifies sensor id, check data within that sensor id only
            return not any(id == target_sensor_id for id in self.sid)

    # getter for points list in format to be used for display
    def get_points_for_display(self, sensor_id) -> list:
        points_list = []
        for i, id in enumerate(self.sid):
            if id == sensor_id:
                points_list.append((self.x[i], self.y[i], self.z[i], self.is_static[i]))
        return points_list

    # TODO: points_for_clustering not working as expected, each radar frame contains points for only 1 sensor at a
    #  time. reformat how its used in main or just delete bc combining points for display works well.
    def points_for_clustering(self) -> list:
        points_list = []
        for i, status in enumerate(self.is_static):
            if status == 0: # if point is not static
                points_list.append((self.x[i], self.y[i], self.z[i])) # for actual z value
                # points_list.append((self.x[i], self.y[i], 0)) # flatten z value
                print(self.sid)
        return points_list

    def get_xyz_coord(self, sensor_id) -> list:
        points_list = []
        for i, id in enumerate(self.sid):
            if id == sensor_id:
                points_list.append((self.x[i], self.y[i], self.z[i]))
        return points_list

    def set_static_points(self, points_list: list) -> None:
        """find a match of (x,y) to self.x and self.y lists and update is_static"""
        if len(points_list) > 0:
            assert len(points_list[0]) == 3, "points_list contains tuples of (x,y,z)"
        for i, (x, y, z) in enumerate(zip(self.x, self.y, self.z)):
            if (x, y, z) in points_list:
                self.is_static[i] = 1
            else:
                self.is_static[i] = 0

    def frame_transform_coord(self, s1_rotz, s1_rotx, s2_rotz, s2_rotx, 
                        offsetx, offsety, offsetz):
        for i in range(len(self.x)):
            xyz = np.asmatrix(([self.x[i]], [self.y[i]], [self.z[i]]))  # cm to mm
            if self.sid[i] == 1:
                # entry sensor
                xyz_transformed = np.matmul(s1_rotz, np.matmul(s1_rotx, xyz))
                xyz_transformed += np.array([[offsetx], [-offsety], [offsetz]])
            elif self.sid[i] == 2:
                xyz_transformed = np.matmul(s2_rotz, np.matmul(s2_rotx, xyz))
                xyz_transformed += np.array([[-offsetx], [offsety], [offsetz]])
            else:
                print("RadarFrame: Sensor ID not supported")
                raise
            self.x[i] = float(xyz_transformed[0])
            self.y[i] = float(xyz_transformed[1])
            self.z[i] = float(xyz_transformed[2])
