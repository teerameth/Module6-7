import cv2
import numpy as np
import imutils
MIN_MATCHES = 10
# cap = cv2.imread('test1.jpg', 0)
cap = cv2.imread('Picture2.jpg', 0)
model = cv2.imread('Template1.jpg', 0)
cap = imutils.resize(cap, height=500)
model = imutils.resize(model, height=500)
# ORB keypoint detector
orb = cv2.ORB_create()
# create brute force  matcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# Compute model keypoints and its descriptors
kp_model, des_model = orb.detectAndCompute(model, None)
# Compute scene keypoints and its descriptors
kp_frame, des_frame = orb.detectAndCompute(cap, None)
# Match frame descriptors with model descriptors
matches = bf.match(des_model, des_frame)
# Sort them in the order of their distance
matches = sorted(matches, key=lambda x: x.distance)

if len(matches) > MIN_MATCHES:
    # draw first 15 matches.
    show = cv2.drawMatches(model, kp_model, cap, kp_frame,
                          matches[:MIN_MATCHES], 0, flags=2)

    # assuming matches stores the matches found and
    # returned by bf.match(des_model, des_frame)
    # differenciate between source points and destination points
    src_pts = np.float32([kp_model[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    # compute Homography
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    print(M)

    # Draw a rectangle that marks the found model in the frame
    h, w = model.shape
    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
    # project corners into frame
    dst = cv2.perspectiveTransform(pts, M)

    ##fill extracted image
    fill_point = np.array(dst, dtype=np.int32)
    save = cv2.fillConvexPoly(cap, fill_point, 255)

    for item in dst: item[0][0] += len(model[0])
    # connect them with lines
    show = cv2.polylines(show, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

    # show result
    cv2.imshow('frame', show)
    # cv2.imwrite('show.jpg', show)
    # cv2.imwrite('save.jpg', save)
    cv2.waitKey(0)
else:
    print ("Not enough matches have been found")