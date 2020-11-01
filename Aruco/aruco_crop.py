import cv2
import numpy as np
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv2.aruco.drawMarker(dictionary, 38, 200, markerImage, 1)
# cv2.imshow("marker33", markerImage)
# cv2.waitKey(0)

cameraMatrix = np.array([[1382.0928644688488, 0.0, 782.1348393366559], [0.0, 1375.5847245596997, 443.04280651276923], [0.0, 0.0, 1.0]], np.float32)
dist = np.array([[-0.009807896000630457, 0.8199792579806928, -0.004898059176512277, -0.005541250200228532, -5.527212026851566]])
rvec = np.array([-0.0, 0.0, -0.0])
tvec = np.array([-0.0, 0.0, -0.0])
cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)
parameters =  cv2.aruco.DetectorParameters_create()

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=0.04, markerSeparation=0.01, dictionary=dictionary)
img = board.draw(outSize=(1000, 1000))
cv2.imshow("Marker Plane", img)
# cv2.waitKey(0)

while True:
    _, frame = cap.read()

    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if markerIds is not None:
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        valid = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
        if valid: cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
        for corner, id in zip(markerCorners, markerIds):
            points = [(int(point[0]), int(point[1])) for point in corner[0]]
            ids = id[0]
            pts = np.array(points, np.int32)
            cv2.fillPoly(frame, [pts], 255)

    cv2.imshow("Preview", frame)
    # cv2.imshow("marker33", markerImage)
    cv2.waitKey(10)