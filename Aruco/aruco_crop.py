import cv2
import numpy as np
import math
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
cap.set(3, 1280)
cap.set(4, 720)
parameters =  cv2.aruco.DetectorParameters_create()

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=0.039, markerSeparation=0.01, dictionary=dictionary)
# img = board.draw(outSize=(1000, 1000))
# cv2.imshow("Marker Plane", img)
# cv2.imwrite("MarkerPlane.png", img)

# rotate a markers corners by rvec and translate by tvec if given
# input is the size of a marker.
# In the markerworld the 4 markercorners are at (x,y) = (+- markersize/2, +- markersize/2)
# returns the rotated and translated corners and the rotation matrix
def rotate_marker_corners(rvec, markersize, tvec = None):

    mhalf = markersize / 2.0
    # convert rot vector to rot matrix both do: markerworld -> cam-world
    mrv, jacobian = cv2.Rodrigues(rvec)

    #in markerworld the corners are all in the xy-plane so z is zero at first
    X = mhalf * mrv[:,0] #rotate the x = mhalf
    Y = mhalf * mrv[:,1] #rotate the y = mhalf
    minusX = X * (-1)
    minusY = Y * (-1)

    # calculate 4 corners of the marker in camworld. corners are enumerated clockwise
    markercorners = []
    markercorners.append(np.add(minusX, Y)) #was upper left in markerworld
    markercorners.append(np.add(X, Y)) #was upper right in markerworld
    markercorners.append(np.add( X, minusY)) #was lower right in markerworld
    markercorners.append(np.add(minusX, minusY)) #was lower left in markerworld
    # if tvec given, move all by tvec
    if tvec is not None:
        C = tvec #center of marker in camworld
        for i, mc in enumerate(markercorners):
            markercorners[i] = np.add(C,mc) #add tvec to each corner
    #print('Vec X, Y, C, dot(X,Y)', X,Y,C, np.dot(X,Y)) # just for debug
    markercorners = np.array(markercorners,dtype=np.float32) # type needed when used as input to cv2
    return markercorners, mrv
def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
while True:
    _, frame = cap.read()
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if markerIds is not None:
        ret, _, _ = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
        if ret:
            # cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
            # cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec + np.array([0.05, 0.0, 0.0]), length=0.1)
            # Find the rotation and translation vectors.
            X = np.array([-0.1, 0.0, 0.0])
            Y = np.array([0.0, -0.1, 0.0])
            ret,rvecs, tvecs = cv2.solvePnP([X], markerCorners, cameraMatrix, dist)
            # project 3D points to image plane
            axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cameraMatrix, dist)
            frame = draw(frame,markerCorners,imgpts)
            # flag, rvec, tvec = cv2.solvePnP(X, tvec, cameraMatrix, dist)
            print("{:.2f}, {:.2f}, {:.2f}".format(tvec[0], tvec[1], tvec[2]))
            # print("{:.2f}, {:.2f}, {:.2f}".format(rvec[0], rvec[1], rvec[2]))
            # cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec_board, tvec=tvec + np.dot(X, rvec_board.T), length=0.1)
            cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=np.array([0.0, 0.0, 0.0]), tvec=tvec + X, length=0.1)
            cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec, length=0.1)
            
        ## Fill Marker ##
        for corner, id in zip(markerCorners, markerIds):
            points = [(int(point[0]), int(point[1])) for point in corner[0]]
            ids = id[0]
            pts = np.array(points, np.int32)
            # cv2.fillPoly(frame, [pts], 255)
        ## Perspective Crop


    cv2.imshow("Preview", frame)
    # cv2.imshow("marker33", markerImage)
    cv2.waitKey(10)
