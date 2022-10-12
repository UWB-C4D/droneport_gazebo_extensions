# Tested using VLC :
# vlc -vvv video_file.xyz --sout udp:127.0.0.1:5000
import cv2

#cap = cv2.VideoCapture('udp://127.0.0.1:5600', cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert ! appsink emit-signals=true sync=false max-buffers=2 drop=true", cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print('VideoCapture not opened')
    exit(-1)

while True:
    ret, frame = cap.read()
    print(frame.shape)
    if not ret:
        print('frame empty')
        break

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
