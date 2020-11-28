import cv2
import numpy as np
import math
from transform import four_point_transform, order_points
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv2.aruco.drawMarker(dictionary, 38, 200, markerImage, 1)
# cv2.imshow("marker33", markerImage)
# cv2.waitKey(0)

# cameraMatrix = np.array([[1438.4337197221366, 0.0, 934.4226787746103], [0.0, 1437.7513778197347, 557.7771398018671], [0.0, 0.0, 1.0]], np.float32) # Module
cameraMatrix = np.array([[1395.3709390074625, 0.0, 984.6248356317226], [0.0, 1396.2122002126725, 534.9517311724618], [0.0, 0.0, 1.0]], np.float32) # Humanoid
# cameraMatrix = np.array([[852.6434105992806, 0.0, 398.3286136737032], [0.0, 860.8765484709088, 302.00038413294385], [0.0, 0.0, 1.0]], np.float32) # ESP32
# dist = np.array([[0.07229278436610362, -0.5836205675336522, 0.0003932499370206642, 0.0002754754987376089, 1.7293977700105942]]) # Module
dist = np.array([[0.1097213194870457, -0.1989645299789654, -0.002106454674127449, 0.004428959364733587, 0.06865838341764481]]) # Humanoid
# dist = np.array([[0.02220329099612066, 0.13530759611493004, -0.0041870520396677805, 0.007599954530058233, -0.4722284261198788]]) # ESP32
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only

cap = cv2.VideoCapture(cv2.CAP_DSHOW)
codec = 0x47504A4D  # MJPG
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(3, 1920)
cap.set(4, 1080)

# cap = cv2.VideoCapture("A.mp4")

parameters =  cv2.aruco.DetectorParameters_create()
parameters.cornerRefinementMethod = 2
# markerLength=0.039 # real
# markerSeparation=0.0975 # real
markerLength = 0.04
markerSeparation = 0.01
# board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=0.039, markerSeparation=0.0975, dictionary=dictionary) # real
board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=markerLength, markerSeparation=markerSeparation, dictionary=dictionary)
# img = board.draw(outSize=(1000, 1000))
# cv2.imshow("Marker Plane", img)
# cv2.imwrite("MarkerPlane.png", img)

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

while True:
    _, frame = cap.read()
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    # try:
    #     rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, dist)
    #     for i in range(len(markerCorners)): cv2.aruco.drawAxis(frame, cameraMatrix, dist, rvecs[i], tvecs[i], 0.1)
    # except: pass
    if markerIds is not None:
        ret, _, _ = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
        if ret:
            # cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1) # origin
            T_marker = np.array([markerLength, markerLength, 0.0])
            A = np.array([0.0, 0.0, 0.0]) + T_marker
            B = np.array([0.4 + markerSeparation, 0.0, 0.0]) + T_marker
            C = np.array([0.4 + markerSeparation, 0.4 + markerSeparation, 0.0]) + T_marker
            D = np.array([0.0, 0.4 + markerSeparation, 0.0]) + T_marker
            ### Find Transformation Matrix ###
            rotM = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec, rotM, jacobian = 0)
            rotM = rotM.T
            ### Map to image coordinate ###
            pts, jac = cv2.projectPoints(np.float32([A, B, C, D]).reshape(-1,3), rvec, tvec, cameraMatrix, dist)
            pts = np.array([tuple(pts[i].ravel()) for i in range(4)], dtype = "float32")
            pts = order_points(pts)
            ### Draw axis ###
            for point in [A, B, C, D]: cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec + np.dot(point, rotM), length=0.1)
            ### Draw work space ###
            # drawBox(frame, rvec, tvec + np.dot(A, rotM.T), size=0.4 + markerSeparation)

        ## Fill Marker ##
        for corner, id in zip(markerCorners, markerIds):
            points = [(int(point[0]), int(point[1])) for point in corner[0]]
            ids = id[0]
            pts1 = np.array(points, np.int32)
            cv2.fillPoly(frame, [pts1], 255)
        ## Perspective Crop ##
        try:
            warped = four_point_transform(frame, pts)
            warped = cv2.resize(warped, (800, 800))
            valid_mask = four_point_transform(np.ones(frame.shape[:2], dtype="uint8") * 255, pts)
            valid_mask = cv2.resize(valid_mask, (800, 800))
            cv2.imshow("Warped", warped)
            cv2.imshow("Valid", valid_mask)
        except: pass
    cv2.imshow("Preview", frame)
    # cv2.imshow("marker33", markerImage)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()