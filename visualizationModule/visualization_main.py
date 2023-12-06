from .preprocess import rot_mtx_entry, rot_mtx_exit, load_data_tlv
from .radar_points import StaticPoints, RadarFrame
from .radar_clustering import *
import yaml, cv2

with open("visualizationModule/visualconfig.yaml", "r") as f:
    v_config = yaml.safe_load(f)

# initialization of gate setup parameters
alpha = v_config["SensorAngles"]["alpha"]
beta = v_config["SensorAngles"]["beta"]
# distance of sensor from gate centre, positive in mm
offsetx = v_config["SensorOffsets"]["offsetx"]
offsety = v_config["SensorOffsets"]["offsety"]
offsetz = v_config["SensorOffsets"]["offsetz"]
# initialize rotation matrices
s1_rotz, s1_rotx = rot_mtx_entry(alpha, beta)
s2_rotz, s2_rotx = rot_mtx_exit(alpha, beta)
# visualization parameters
rad_cam_offset = v_config["rad_cam_offset"]
scalemm2px = v_config["scalemm2px"]
wait_ms = v_config["wait_ms"]
slider_xoffset = v_config["TrackbarDefaults"]["slider_xoffset"]
slider_yoffset = v_config["TrackbarDefaults"]["slider_yoffset"]
xy_trackbar_scale = v_config["TrackbarDefaults"]["xy_trackbar_scale"]

# ------------------ CV2 SUPPORT FUNCTIONS ------------------ #

# BGR colours for drawing points on frame (OpenCV)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
BLUE = (255, 0, 0)
RED = (0, 0, 255)
ORANGE = (0, 165, 255)


def washout(color, factor=0.2):
    # create washed out color
    return (int(color[0] * factor), int(color[1] * factor), int(color[2] * factor))


def x_trackbar_callback(*args):
    # updates global x offset by trackbar value
    global slider_xoffset
    slider_xoffset = cv2.getTrackbarPos("x offset", "Live Camera Feed")


def y_trackbar_callback(*args):
    # updates global y offset by trackbar value
    global slider_yoffset
    slider_yoffset = cv2.getTrackbarPos("y offset", "Live Camera Feed")


def scale_callback(*args):
    # multiplies x and y by scale value from trackbar
    global xy_trackbar_scale
    xy_trackbar_scale = cv2.getTrackbarPos("scale %", "Live Camera Feed") / 100


# draw gate at top left of window, with width and height of gate.
# Scale to match gate location with trackbar - returns valid display region
def draw_gate_topleft(frame):
    # initial coords at top left corner (0,0)
    rect_start = ((slider_xoffset), (slider_yoffset))
    # rect end initial coords are based on the physical width and height of the gate
    rect_end = (
        (int(offsetx * 2 * scalemm2px * xy_trackbar_scale) + slider_xoffset),
        (int(offsety * 2 * scalemm2px * xy_trackbar_scale) + slider_yoffset),
    )
    cv2.rectangle(frame, rect_start, rect_end, BLUE, 2)
    return rect_start, rect_end


def remove_points_outside_gate(points, rect_start, rect_end) -> list:
    """Remove points that are outside the gate area.
    Returns a list of points that are inside the gate area."""
    points_in_gate = []
    for coord in points:
        x = int((coord[0] + offsetx) * scalemm2px)
        y = int((-coord[1] + offsety) * scalemm2px)
        x = int(x * xy_trackbar_scale)
        y = int(y * xy_trackbar_scale)
        x += slider_xoffset
        y += slider_yoffset
        if x < rect_start[0] or x > rect_end[0]:
            continue
        if y < rect_start[1] or y > rect_end[1]:
            continue
        points_in_gate.append(coord)
    return points_in_gate


def draw_radar_points(frame, points, sensor_id):
    if sensor_id == 1:
        color = GREEN
    elif sensor_id == 2:
        color = YELLOW
    else:
        raise
    for coord in points:
        x = int((coord[0] + offsetx) * scalemm2px)
        y = int((-coord[1] + offsety) * scalemm2px)  # y axis is flipped
        z = int(coord[2] * scalemm2px)  # z is not used
        static = coord[3]

        # xy modifications from trackbar controls
        x = int(x * xy_trackbar_scale)
        y = int(y * xy_trackbar_scale)
        x += slider_xoffset
        y += slider_yoffset
        if static:
            cv2.circle(frame, (x, y), 4, washout(color), -1)
        else:
            cv2.circle(frame, (x, y), 4, color, -1)


def draw_clustered_points(frame, processed_centroids, color=RED):
    for cluster in processed_centroids:
        x = int((int(cluster["x"] + offsetx) * scalemm2px))
        y = int((int(-cluster["y"] + offsety) * scalemm2px))  # y axis is flipped
        # z = int(coord[2] * scalemm2px)  # z is not used
        # static = coord[3]
        # xy modifications from trackbar controls
        x = int(x * xy_trackbar_scale)
        y = int(y * xy_trackbar_scale)
        x += slider_xoffset
        y += slider_yoffset
        cv2.circle(frame, (x, y), 10, color, -1)


def draw_bbox(frame, centroids, cluster_point_cloud):
    for i in enumerate(centroids):
        x1, y1, x2, y2 = cluster_bbox(cluster_point_cloud, i[0])
        # convert mm to px
        x1, y1, x2, y2 = (
            int(x1 + offsetx) * scalemm2px,
            int(-y1 + offsety) * scalemm2px,
            int(x2 + offsetx) * scalemm2px,
            int(-y2 + offsety) * scalemm2px,
        )
        # modify based on trackbar
        x1, y1, x2, y2 = (
            int(x1 * xy_trackbar_scale) + slider_xoffset,
            int(y1 * xy_trackbar_scale) + slider_yoffset,
            int(x2 * xy_trackbar_scale) + slider_xoffset,
            int(y2 * xy_trackbar_scale) + slider_yoffset,
        )
        object_size, object_height = obj_height(cluster_point_cloud, i[0])
        rect = cv2.rectangle(frame, (x1, y1), (x2, y2), ORANGE, 1)
        size, _ = cv2.getTextSize(
            f"{object_height:.1f} mm", cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
        )
        text_width, text_height = size
        cv2.putText(
            rect,
            f"{object_height:.1f} mm",
            (x1, y1 - text_height - 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            ORANGE,
            2,
        )


def display_control_info(frame, width):
    cv2.putText(
        frame,
        "Controls - 'q': quit  'p': pause",
        (width - 175, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (0, 0, 150),
        1,
    )
    cv2.putText(
        frame,
        "scale/offset gate region with trackbar",
        (width - 217, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (0, 0, 150),
        1,
    )


cv2.namedWindow("Live Camera Feed")
cv2.createTrackbar(
    "x offset", "Live Camera Feed", slider_xoffset, 600, x_trackbar_callback
)
cv2.createTrackbar(
    "y offset", "Live Camera Feed", slider_yoffset, 600, y_trackbar_callback
)
cv2.createTrackbar(
    "scale %", "Live Camera Feed", int(xy_trackbar_scale * 100), 200, scale_callback
)  # *100 and /100 to account for floating point usuability to downscale


def run_visualization(cam_payload, radar_payload):
    if cam_payload is None:
        print("No camera payload received.")
        return
    if not hasattr(run_visualization, "s1_static"):
        run_visualization.s1_static = StaticPoints()
    if not hasattr(run_visualization, "s2_static"):
        run_visualization.s2_static = StaticPoints()
    if not hasattr(run_visualization, "s1_display_points_prev"):
        run_visualization.s1_display_points_prev = []
    if not hasattr(run_visualization, "s2_display_points_prev"):
        run_visualization.s2_display_points_prev = []

    frame_payload = cam_payload[:-26]  # remove timestamp
    # convert byte array to numpy array for cv2 to read
    frame = np.frombuffer(frame_payload, dtype=np.uint8)
    frame = frame.reshape((480, 640, 3))
    if not hasattr(run_visualization, "window_created"):
        cv2.namedWindow("Live Camera Feed")
        run_visualization.window_created = True
    if frame.size == 0:
        print("Empty frame received.")
        return
    if frame.size != 921600:
        print(f"Invalid frame size of {frame.size} bytes.")
        return
    if frame.ndim != 3:
        print(f"Invalid frame dimension of {frame.ndim}.")
        return
    height, width = frame.shape[:2]
    frame = cv2.resize(frame, (round(width), round(height)))  # reduce frame size
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    height, width = frame.shape[:2]

    # draw gate area and get gate area coordinates
    gate_tl, gate_br = draw_gate_topleft()

    # take points in current RADAR frame
    radar_frame = RadarFrame(load_data_tlv(radar_payload))
    if radar_frame.is_empty():
        return
    radar_frame.transform_coord(
        s1_rotz, s1_rotx, s2_rotz, s2_rotx, offsetx, offsety, offsetz
    )
    print(f"Radar frame transformed.\n")
    print(f"radar_frame: {radar_frame}")

    # update static points, prepare for display
    s1_display_points = []
    s2_display_points = []
    if not radar_frame.is_empty(target_sensor_id=1):
        run_visualization.s1_static.update(radar_frame.get_xyz_coord(sensor_id=1))
        radar_frame.set_static_points(run_visualization.s1_static.get_static_points())
        s1_display_points = radar_frame.get_points_for_display(sensor_id=1)

    if not radar_frame.is_empty(target_sensor_id=2):
        run_visualization.s2_static.update(radar_frame.get_xyz_coord(sensor_id=2))
        radar_frame.set_static_points(run_visualization.s2_static.get_static_points())
        s2_display_points = radar_frame.get_points_for_display(sensor_id=2)

    # remove points that are out of gate area, if configured
    if v_config["remove_noise"]:
        s1_display_points = remove_points_outside_gate(
            s1_display_points, gate_tl, gate_br
        )
        s2_display_points = remove_points_outside_gate(
            s2_display_points, gate_tl, gate_br
        )

    # retain previous frame if no new points
    if not s1_display_points:
        s1_display_points = run_visualization.s1_display_points_prev
    else:
        run_visualization.s1_display_points_prev = s1_display_points
    if not s2_display_points:
        s2_display_points = run_visualization.s2_display_points_prev
    else:
        run_visualization.s2_display_points_prev = s2_display_points

    # get all non-static points and cluster
    s1_s2_combined = [
        values[:-1]
        for values in s1_display_points + s2_display_points
        if values[-1] == 0
    ]
    if len(s1_s2_combined) > 1:
        processor = ClusterProcessor(
            eps=250, min_samples=4
        )  # default: eps=400, min_samples=5 --> eps is in mm
        centroids, cluster_point_cloud = processor.cluster_points(
            s1_s2_combined
        )  # get the centroids of each
        # cluster and their associated point cloud
        draw_clustered_points(
            frame, centroids
        )  # may not be in the abs center of bbox --> "center of mass", not area
        # centroid.
        draw_clustered_points(
            frame, cluster_point_cloud, color=BLUE
        )  # highlight the points that belong to the detected
        # obj
        draw_bbox(
            frame, centroids, cluster_point_cloud
        )  # draw the bounding box of each cluster

    # draw points on frame
    if s1_display_points:
        draw_radar_points(frame, s1_display_points, sensor_id=1)
    if s2_display_points:
        draw_radar_points(frame, s2_display_points, sensor_id=2)

    display_control_info(frame, width)

    # after drawing points on frames, imshow the frames
    cv2.imshow("Live Camera Feed", frame)
    cv2.waitKey(0.01)
    # # Key controls
    # key = cv2.waitKey(wait_ms) & 0xFF
    # if key == ord("q"):  # quit program if 'q' is pressed
    #     break
    # elif key == ord("p"):  # pause/unpause program if 'p' is pressed
    #     cv2.waitKey(0)

    # cv2.destroyAllWindows()
