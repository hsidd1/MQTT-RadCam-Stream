import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()
    cv2.imshow("frame", frame)
    print(frame.shape)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
