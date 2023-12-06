import cv2
import yaml

"""
Script to write a sample frame to file for testing purposes.
Bytearray of the frame is written to data/sample_frame.txt since this
object is extremely large
"""

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

if __name__ == "__main__":
    cap = cv2.VideoCapture(config["Files"]["video_file"])
    ret, frame = cap.read()
    with open("data/sample_frame.txt", "a") as f:
        f.write(str(bytearray(frame)))
    print("Sample frame written to data/sample_frame.txt")
    cap.release()
    cv2.destroyAllWindows()
