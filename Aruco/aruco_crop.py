import cv2
import numpy as np
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv2.aruco.drawMarker(dictionary, 38, 200, markerImage, 1)
# cv2.imshow("marker33", markerImage)
# cv2.waitKey(0)

cameraMatrix = np.array([[1438.4337197221366, 0.0, 934.4226787746103], [0.0, 1437.7513778197347, 557.7771398018671], [0.0, 0.0, 1.0]], np.float32)
dist = np.array([[0.07229278436610362, -0.5836205675336522, 0.0003932499370206642, 0.0002754754987376089, 1.7293977700105942]])
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only

cap = cv2.VideoCapture(cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(3, 1920)
cap.set(4, 1080)
parameters =  cv2.aruco.DetectorParameters_create()

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=0.04, markerSeparation=0.01, dictionary=dictionary)
img = board.draw(outSize=(1000, 1000))
cv2.imshow("Marker Plane", img)
# cv2.imwrite("MarkerPlane.png", img)

while True:
    _, frame = cap.read()

    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if markerIds is not None:
        cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        valid = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
        if valid:
            print(tvec)
            cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
            cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec + np.array([0.05, 0.0, 0.0]), length=0.1)
        ## Fill Marker ##
        for corner, id in zip(markerCorners, markerIds):
            points = [(int(point[0]), int(point[1])) for point in corner[0]]
            ids = id[0]
            pts = np.array(points, np.int32)
            cv2.fillPoly(frame, [pts], 255)
        ## Perspective Crop


    cv2.imshow("Preview", frame)
    # cv2.imshow("marker33", markerImage)
    cv2.waitKey(10)