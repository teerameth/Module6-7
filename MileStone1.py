import cv2
import numpy as np
import imutils
import v4l2capture
import select
import argparse

width, height = 1920, 1080
# width, height = 1280, 720
full_screen = False
WINDOW_NAME = "Preview"
def imshow_fullscreen(winname, img):
    img = imutils.resize(img, height=1080)
    cv2.namedWindow (winname, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty (winname, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow (winname, img)

cap = v4l2capture.Video_device("/dev/video0")
size_x, size_y = cap.set_format(width, height, fourcc='MJPG')
cap.create_buffers(1)
cap.queue_all_buffers()
cap.start()
while True:
    select.select((cap,), (), ())
    image_data = cap.read_and_queue()
    img = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)

    # Initiate ORB detector
    orb = cv2.ORB_create()
    # find the keypoints with ORB
    kp = orb.detect(img, None)
    # compute the descriptors with ORB
    kp, des = orb.compute(img, kp)
    # draw only keypoints location,not size and orientation
    img = cv2.drawKeypoints(img, kp, img, color=(0,255,0), flags=0)

    pts_src = np.array([[141, 131], [480, 159], [493, 630],[64, 601]])
    pts_src = np.array([[0, 0],[0, 1920],[1080, 1920],[1080, 0]])
    pts_dst = np.array([[318, 256],[534, 372],[316, 670],[73, 473]])
    # pts_dst = np.array([[0, 0],[0, 1920],[1080, 1920],[1080, 0]])
    h, status = cv2.findHomography(pts_src, pts_dst)
    img = cv2.warpPerspective(img, h, (width, height))
    
    imshow_fullscreen(WINDOW_NAME, img)
    key = cv2.waitKey(1)
    if key == ord('q'):
      break
cap.close()
cv2.destroyAllWindows()