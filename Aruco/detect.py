import cv2
import numpy as np
import math
from transform import four_point_transform, order_points
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
cameraMatrix = np.array([[1430.4798932831916, 0.0, 919.793593267191], [0.0, 1429.5119845386027, 570.9534919974565], [0.0, 0.0, 1.0]], np.float32) # Module
# cameraMatrix = np.array([[1361.3720519541948, 0.0, 988.234800503673], [0.0, 1358.359480587064, 528.3772257989573], [0.0, 0.0, 1.0]], np.float32) # Humanoid
# cameraMatrix = np.array([[852.6434105992806, 0.0, 398.3286136737032], [0.0, 860.8765484709088, 302.00038413294385], [0.0, 0.0, 1.0]], np.float32) # ESP32
dist = np.array([[0.06895705411990097, -0.9617085061810868, -0.0033372226544416596, -0.00036649375857501104, 3.4072884355893542]]) # Module
# dist = np.array([[0.02220329099612066, 0.13530759611493004, -0.0041870520396677805, 0.007599954530058233, -0.4722284261198788]]) # Humanoid
# dist = np.array([[0.02220329099612066, 0.13530759611493004, -0.0041870520396677805, 0.007599954530058233, -0.4722284261198788]]) # ESP32
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only
# cap = cv2.VideoCapture(cv2.CAP_DSHOW)
# codec = 0x47504A4D  # MJPG
# cap.set(cv2.CAP_PROP_FPS, 30.0)
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
# cap.set(3, 1920)
# cap.set(4, 1080)

cap = cv2.VideoCapture("A.mp4")

parameters =  cv2.aruco.DetectorParameters_create()
markerLength=0.039 # real
markerSeparation=0.0975 # real

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=markerLength, markerSeparation=markerSeparation, dictionary=dictionary)
ret, frame = cap.read()
while ret:
    ret, frame = cap.read()
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners=markerCorners, markerLength=0.04, cameraMatrix=cameraMatrix, distCoeffs=dist)
    for o in _objPoints:
        print(o)
    for i in range(len(markerCorners)):
        cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvecs[i], tvec=tvecs[i], length=0.02)
    for corner, id in zip(markerCorners, markerIds):
            points = [(int(point[0]), int(point[1])) for point in corner[0]]
            ids = id[0]
            pts1 = np.array(points, np.int32)
            cv2.fillPoly(frame, [pts1], 255)
    cv2.imshow("Preview", frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()