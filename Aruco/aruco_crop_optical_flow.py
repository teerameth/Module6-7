# Farneback’s Dense Optical Flow algorithm
import cv2
import numpy as np
import math
import time
from transform import four_point_transform, order_points
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

cameraMatrix = np.array([[1395.3709390074625, 0.0, 984.6248356317226], [0.0, 1396.2122002126725, 534.9517311724618], [0.0, 0.0, 1.0]], np.float32) # Humanoid
dist = np.array([[0.1097213194870457, -0.1989645299789654, -0.002106454674127449, 0.004428959364733587, 0.06865838341764481]]) # Humanoid
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only

# cap = cv2.VideoCapture(cv2.CAP_DSHOW)
# codec = 0x47504A4D  # MJPG
# cap.set(cv2.CAP_PROP_FPS, 30.0)
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
# cap.set(3, 1920)
# cap.set(4, 1080)

cap = cv2.VideoCapture("J.mp4")
cuda_stream = cv2.cuda_Stream()
parameters =  cv2.aruco.DetectorParameters_create()
# parameters(doCornerRefinement=True)
# markerLength=0.039 # real
# markerSeparation=0.0975 # real
markerLength = 0.04
markerSeparation = 0.01
# board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=0.039, markerSeparation=0.0975, dictionary=dictionary) # real
board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=markerLength, markerSeparation=markerSeparation, dictionary=dictionary)
backSub = cv2.cuda.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)
# backSub = cv2.createBackgroundSubtractorKNN(history=30, dist2Threshold=400.0, detectShadows=True)
img_list, mask_list = [], []

i=0
frame_counter = 0
mode = True
N = 1000
for j in range(N):
    _, frame = cap.read()
    original = frame.copy()
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if markerIds is not None:
        ret, _, _ = cv2.aruco.estimatePoseBoard(corners=markerCorners, ids=markerIds, board=board, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec)
        if ret:
            cv2.imshow("Preview", frame)
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
            for point in [A, B, C, D]: cv2.aruco.drawAxis(image=frame, cameraMatrix=cameraMatrix, distCoeffs=dist, rvec=rvec, tvec=tvec + np.dot(point, rotM.T), length=0.1)
            ### Draw work space ###
            # drawBox(frame, rvec, tvec + np.dot(A, rotM.T), size=0.4 + markerSeparation)
        ## Fill Marker ##
        for corner, id in zip(markerCorners, markerIds):
            points = [(int(point[0]), int(point[1])) for point in corner[0]]
            ids = id[0]
            pts1 = np.array(points, np.int32)
            cv2.fillPoly(frame, [pts1], 255)
        ## Perspective Crop ##
        warped = four_point_transform(original, pts)
        warped = cv2.resize(warped, (800, 800))
        valid_mask = four_point_transform(np.ones(original.shape[:2], dtype="uint8") * 255, pts)
        valid_mask = cv2.resize(valid_mask, (800, 800))
        cv2.imshow("Warped", warped)
        cv2.imshow("Valid", valid_mask)

        if j == 0:
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(warped)
            previous_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gpu_previous = cv2.cuda_GpuMat()
            gpu_previous.upload(previous_frame)
            # create gpu_hsv output for optical flow
            gpu_hsv = cv2.cuda_GpuMat(gpu_frame.size(), cv2.CV_32FC3)
            gpu_hsv_8u = cv2.cuda_GpuMat(gpu_frame.size(), cv2.CV_8UC3)

            gpu_h = cv2.cuda_GpuMat(gpu_frame.size(), cv2.CV_32FC1)
            gpu_s = cv2.cuda_GpuMat(gpu_frame.size(), cv2.CV_32FC1)
            gpu_v = cv2.cuda_GpuMat(gpu_frame.size(), cv2.CV_32FC1)
            # set saturation to 1
            gpu_s.upload(np.ones_like(previous_frame, np.float32))
        else:
            frame = warped
            # upload frame to GPU
            gpu_frame.upload(frame)
            # resize frame
            gpu_frame = cv2.cuda.resize(gpu_frame, (960, 540))
            # convert to gray
            gpu_current = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)
            # create optical flow instance
            gpu_flow = cv2.cuda_FarnebackOpticalFlow.create(5, 0.5, False, 15, 3, 5, 1.2, 0)
            # calculate optical flow
            gpu_flow = cv2.cuda_FarnebackOpticalFlow.calc(gpu_flow, gpu_previous, gpu_current, None)
            gpu_flow_x = cv2.cuda_GpuMat(gpu_flow.size(), cv2.CV_32FC1)
            gpu_flow_y = cv2.cuda_GpuMat(gpu_flow.size(), cv2.CV_32FC1)
            cv2.cuda.split(gpu_flow, [gpu_flow_x, gpu_flow_y])
            # convert from cartesian to polar coordinates to get magnitude and angle
            gpu_magnitude, gpu_angle = cv2.cuda.cartToPolar(gpu_flow_x, gpu_flow_y, angleInDegrees=True,)
            # set value to normalized magnitude from 0 to 1
            gpu_v = cv2.cuda.normalize(gpu_magnitude, 0.0, 1.0, cv2.NORM_MINMAX, -1)
            # get angle of optical flow
            angle = gpu_angle.download()
            angle *= (1 / 360.0) * (180 / 255.0)
            # set hue according to the angle of optical flow
            gpu_h.upload(angle)
            # merge h,s,v channels
            cv2.cuda.merge([gpu_h, gpu_s, gpu_v], gpu_hsv)
            # multiply each pixel value to 255
            gpu_hsv.convertTo(cv2.CV_8U, 255.0, gpu_hsv_8u, 0.0)
            # convert hsv to bgr
            gpu_bgr = cv2.cuda.cvtColor(gpu_hsv_8u, cv2.COLOR_HSV2BGR)
            # send original frame from GPU back to CPU
            frame = gpu_frame.download()
            # send result from GPU back to CPU
            bgr = gpu_bgr.download()
            # update previous_frame value
            gpu_previous = gpu_current

        
    # cv2.imshow("marker33", markerImage)
    key = cv2.waitKey(1)
    if key == 27:
        break
    if key == ord('m'):
        mode = not mode
    if key == ord(' '):
        cv2.imwrite(str(i) + ".jpg", warped)
        i+=1
    if key == ord('g'):
        result = calculateBG(img_list, mask_list)
        cv2.imshow("BG", result)
        cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()