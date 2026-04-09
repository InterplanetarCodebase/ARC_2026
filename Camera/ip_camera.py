import cv2

url = "rtsp://admin:Interplanetar123@192.168.1.141:554/live/0/MAIN"

cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

while True:
    ret, frame = cap.read()

    if not ret:
        print("Frame not received")
        break

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()