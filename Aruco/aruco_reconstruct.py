import cv2
import numpy as np
import os
import glob
import imutils
from transform import four_point_transform, order_points
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
cameraMatrix = np.array([[1438.4337197221366, 0.0, 934.4226787746103], [0.0, 1437.7513778197347, 557.7771398018671], [0.0, 0.0, 1.0]], np.float32) # Module
dist = np.array([[0.02220329099612066, 0.13530759611493004, -0.0041870520396677805, 0.007599954530058233, -0.4722284261198788]]) # ESP32
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only

parameters =  cv2.aruco.DetectorParameters_create()

markerLength = 0.04
markerSeparation = 0.01

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=markerLength, markerSeparation=markerSeparation, dictionary=dictionary)

# backSub = cv2.createBackgroundSubtractorMOG2()

def drawBox(frame, rvec, tvec, size = 0.4):
    objpts = np.float32([[0,0,0], [1,0,0], [1,1,0], [0,1,0],
                         [0,0,1], [1,0,1], [1,1,1], [0,1,1]]).reshape(-1,3) * size
    imgpts, jac = cv2.projectPoints(objpts, rvec, tvec, cameraMatrix, dist)

    cv2.line(frame, tuple(imgpts[0].ravel()), tuple(imgpts[1].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[1].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[2].ravel()), tuple(imgpts[3].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (0,0,255), 2)

    cv2.line(frame, tuple(imgpts[0].ravel()), tuple(imgpts[0+4].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[1].ravel()), tuple(imgpts[1+4].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[2].ravel()), tuple(imgpts[2+4].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[3].ravel()), tuple(imgpts[3+4].ravel()), (0,0,255), 2)

    cv2.line(frame, tuple(imgpts[0+4].ravel()), tuple(imgpts[1+4].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[1+4].ravel()), tuple(imgpts[2+4].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[2+4].ravel()), tuple(imgpts[3+4].ravel()), (0,0,255), 2)
    cv2.line(frame, tuple(imgpts[3+4].ravel()), tuple(imgpts[0+4].ravel()), (0,0,255), 2) 


images = glob.glob('./Aruco/images/module/test/*.jpg')
valid_mask_list = []
warped_list = []
for fname in images:
    frame = cv2.imread(fname)
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if markerIds is not None:
        ret, _, _ = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
        if ret:
            # cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1) # origin
            T_marker = np.array([markerLength, markerLength, 0.0])
            A = np.array([0.0, 0.0, 0.0]) + T_marker
            B = np.array([0.4 + markerSeparation, 0.0, 0.0]) + T_marker
            C = np.array([0.4 + markerSeparation, 0.4 + markerSeparation, 0.0]) + T_marker
            D = np.array([0.0, 0.4 + markerSeparation, 0.0]) + T_marker
            ### Find Transformatio Matrix ###
            rotM = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec, rotM, jacobian = 0)
            ### Map to image coordinate ###
            pts, jac = cv2.projectPoints(np.float32([A, B, C, D]).reshape(-1,3), rvec, tvec, cameraMatrix, dist)
            pts = np.array([tuple(pts[i].ravel()) for i in range(4)], dtype = "float32")
            pts = order_points(pts)
            ### Draw axis ###
            # for point in [A, B, C, D]: cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec + np.dot(point, rotM.T), length=0.1)
        warped = four_point_transform(frame, pts)
        warped = cv2.resize(warped, (800, 800))
        valid_mask = four_point_transform(np.ones(frame.shape[:2], dtype="uint8"), pts)
        valid_mask = cv2.resize(valid_mask, (800, 800))
        cv2.imshow("Warped", warped)
        cv2.imshow("Valid", valid_mask*255)
        # fgMask = backSub.apply(warped)
        # cv2.imshow('FG Mask', fgMask)

        valid_mask_list.append(valid_mask)
        warped_list.append(warped)
    cv2.imshow("Preview", imutils.resize(frame, height = 800))
    key = cv2.waitKey(0)
    if key == 27:
        break
### Find mean of every frame ###

# cap.relase()
cv2.destroyAllWindows()