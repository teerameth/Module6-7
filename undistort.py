import cv2
import numpy as np

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

_, frame = cap.read()
h,  w = frame.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))


while True:
    _, frame = cap.read()
    original = frame.copy()
    dst = cv2.undistort(frame, cameraMatrix, dist, None, newcameramtx) # undistort
    # crop the image
    x, y, w, h = roi
    frame = dst[y:y+h, x:x+w]

    cv2.imshow("Preview", frame)
    cv2.imshow("dst", dst)
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()