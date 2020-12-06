import cv2
import numpy as np

def rotate_image_white_edge(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = np.ones_like(image)*255
    mask = cv2.warpAffine(mask, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = cv2.bitwise_not(mask)
    result = cv2.bitwise_or(mask, result)
    result_edge = cv2.Canny(result, 50, 200)
    mask = cv2.erode(cv2.bitwise_not(mask),np.ones((15, 15)),iterations = 1)
    (h, w) = image.shape
    mask_border = cv2.rectangle(np.zeros_like(mask), (int(0.1*w), int(0.1*h)), (int(0.9*w), int(0.9*h)), 255, -1)
    result_edge = cv2.bitwise_and(result_edge, mask)
    result_edge = cv2.bitwise_and(result_edge, mask_border)
    return result_edge

def run(frame, marker, visualize=False):
    template_gray = cv2.cvtColor(marker, cv2.COLOR_BGR2GRAY)
    found = None
    (tH, tW) = template_gray.shape[:2]
    image = frame.copy()
    if len(image.shape) == 3: gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else: gray = image
    found = None
    best_angle = None
    for rotation in np.arange(0, 359, 5):
        resized = image
        template_edge = rotate_image_white_edge(template_gray, rotation)
        # resized = rotate_image(image, rotation)
        r = gray.shape[1] / float(resized.shape[1])
        if resized.shape[0] < tH or resized.shape[1] < tW:  # if the resized image is smaller than the template, then break from the loop
            break
        # detect edges in the resized, grayscale image and apply template
        # matching to find the template in the image
        edged = cv2.Canny(resized, 50, 200)
        result = cv2.matchTemplate(edged, template_edge, cv2.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
        # print(maxVal)
        if visualize:
            clone = np.dstack([edged, edged, edged])
            cv2.rectangle(clone, (maxLoc[0], maxLoc[1]), (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
            cv2.imshow("Template", template_edge)
            cv2.imshow("Visualize", clone)
            cv2.waitKey(1)
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r)
            best_angle = rotation
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    print(found)
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
    if visualize:
        # draw a bounding box around the detected result and display the image
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
        cv2.imshow("A", image)
        # cv2.waitKey(0)
    center = ((startX+endX)/2, (startY+endY)/2)
    mask = np.zeros_like(gray)
    cv2.rectangle(mask, (startX, startY), (endX, endY), 255, -1)
    rot_mat = cv2.getRotationMatrix2D(center, best_angle, 1.0)
    mask = cv2.warpAffine(mask, rot_mat, mask.shape[1::-1], flags=cv2.INTER_LINEAR)
    if visualize:
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)
    return mask, center

if __name__ == "__main__":
    # frame = cv2.imread("X:/segment.png")
    frame = cv2.imread("../img/Real7.png")
    marker = cv2.imread("../img/StartReal1.png")
    run(frame, marker, visualize=True)