import cv2
"""
Extracting random frame as sample for resizing, to account for
not being able to transmit a numpy array over MQTT. To reference
a reshape size, this script is used to extract a frame from the
video file and save it as an image.
"""

frame_num = 50 # extract 50th frame as sample (arbitrary)
def save_frame(video_path, output_path):
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num)
    ret, frame = cap.read()
    if ret:
        # Save the frame as an image
        cv2.imwrite(output_path, frame)
        print(f"Random frame saved to {output_path}")
    else:
        print("Error: Unable to read frame from the video.")
    # Release the video capture object
    cap.release()

if __name__ == "__main__":
    input_video_path = "data/camdata.avi"
    output_image_path = "data/sample_frame.png"
    save_frame(input_video_path, output_image_path)
