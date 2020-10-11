import numpy as np
import cv2

def medianCanny(img, thresh1, thresh2):
    median = np.median(img)
    img = cv2.Canny(img, int(thresh1 * median), int(thresh2 * median))
    return img

class ContourProcessor():
    def set_param(self, image_resolution, image_size, marker_size, marker_size_error):
        self.image_resolution = image_resolution
        self.image_size = image_size
        self.marker_size = marker_size
        self.marker_size_error = marker_size_error
    def get_center(self, c):
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        rect = np.asarray(approx, dtype="float32")
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    def is_contour_rect(self, c): # return True if contour is rectangle
        peri = cv2.arcLength(c, True) # approximate the contour
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        return len(approx) == 4
    def is_contour_marker(self, c):
        if not self.is_contour_rect(c): return False
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        actual_area = cv2.contourArea(approx)
        if actual_area == 0: return False # Avoid divided by zero
        expected_area = (self.marker_size/self.image_size*self.image_resolution)**2
        error = abs(expected_area - actual_area) / actual_area
        if error > self.marker_size_error: return False
        match = 100 * (1 - error)
        # rect = cv2.boundingRect(approx)
        # s_area = rect[2] * rect[3]
        return match
    def get_marker_contours(self, cnts):
        buffer = []
        for i, c in enumerate(cnts):
            match = self.is_contour_marker(c)
            if match:
                dup = False
                for passed in buffer:  # exclude duplicated rect
                    if cv2.pointPolygonTest(passed, self.get_center(c), False) != -1:  # Check if point inside contour
                        dup = True
                if not dup: # If not duplicate rect
                    peri = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                    buffer.append(np.array(approx, dtype=np.int32))
                    print("Match {}%".format(round(match, 2)))
    def 
        return buffer
class Marker():
    def __init__(self, c):
        self.contour = c
class Path():
    def __init__(self, c):
        self.contour = c