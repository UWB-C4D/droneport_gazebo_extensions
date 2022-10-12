import cv2



cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert ! appsink emit-signals=true sync=false max-buffers=16 drop=false", cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 appsink emit-signals=true sync=false", cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()


    if not ret:
        print('frame empty')
        break


    cv2.imshow('aruco_visualization', frame)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break