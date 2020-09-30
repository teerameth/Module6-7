import cv2
import numpy as np
import imutils
MIN_MATCHES = 10
model = cv2.imread('Template1.jpg', 0)
model = imutils.resize(model, height=500)
cam = cv2.VideoCapture(0)
while True:
    ret, cap = cam.read()
    cap = imutils.resize(cap, height=500)

    # ORB keypoint detector
    orb = cv2.ORB_create()
    # create brute force  matcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Compute model keypoints and its descriptors
    kp_model, des_model = orb.detectAndCompute(model, None)
    # Compute scene keypoints and its descriptors
    kp_frame, des_frame = orb.detectAndCompute(cap, None)
    # Match frame descriptors with model descriptors
    try:
        matches = bf.match(des_model, des_frame)
    except : continue
    # Sort them in the order of their distance
    matches = sorted(matches, key=lambda x: x.distance)
    if len(matches) > MIN_MATCHES:
        # draw first 15 matches.
        cap = cv2.drawMatches(model, kp_model, cap, kp_frame,
                              matches[:MIN_MATCHES], 0, flags=2)

        # assuming matches stores the matches found and
        # returned by bf.match(des_model, des_frame)
        # differenciate between source points and destination points
        src_pts = np.float32([kp_model[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        buffer = []
        count = 0
        for i in range(MIN_MATCHES):
            if abs(src_pts[i][0][0] - dst_pts[i][0][0]) < 50 and abs(src_pts[i][0][1] - dst_pts[i][0][1]) < 50: count += 1
            # buffer.append(abs(src_pts[i][0][0] - dst_pts[i][0][0]))
        # print(sum(buffer)/len(buffer))
        print(count)

        # compute Homography
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        # print(float(M[0]), float(M[1]))
        # Draw a rectangle that marks the found model in the frame
        h, w = model.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        # project corners into frame
        dst = cv2.perspectiveTransform(pts, M)
        # print(dst)
        for item in dst: item[0][0] += len(model[0])
        # connect them with lines
        cap = cv2.polylines(cap, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

        # show result
        cv2.imshow('frame', cap)
        # cv2.waitKey(0)
    else:
        print ("Not enough matches have been found")
    cv2.waitKey(10)