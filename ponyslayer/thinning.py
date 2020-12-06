import cv2
import numpy as np
""" From: https://web.archive.org/web/20160314104646/http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/
================
| P9 | P2 | P3 |
|==============|
| P8 | P2 | P4 |
|==============|
| P7 | P6 | P5 |
================
"""

def gouHall(mask):
    (h, w) = mask.shape[:2]
    binary = mask/255
    canvas = np.zeros(mask.shape[:2])
    point_deleted = True
    iteration_count = 0
    canvas_count = 0
    while point_deleted:
        point_deleted = False
        iteration_count += 1
        for y in range(1, h-1):
            for x in range(1, w-1):
                p2 = binary[y-1][x]
                p3 = binary[y-1][x+1]
                p4 = binary[y][x+1]
                p5 = binary[y+1][x+1]
                p6 = binary[y+1][x]
                p7 = binary[y+1][x-1]
                p8 = binary[y][x-1]
                p9 = binary[y-1][x-1]
                C = (not p2 and (p3 or p4)) + (not p4 and (p5 or p6)) + (not p6 and (p7 or p8)) + (not p8 and (p9 or p2))
                N1 = (p9 or p2) + (p3 or p4) + (p5 or p6) + (p7 or p8)
                N2 = (p2 or p3) + (p4 or p5) + (p6 or p7) + (p8 or p9)
                N = min((N1, N2))
                m = (p6 or p7 or not p9) and p8 if iteration_count % 2 == 0 else (p2 or p3 or not p5) and p4
                # print(C, N, m)
                if (C == 1 and N >= 2 and N <= 3) and m == 0:
                    canvas[y][x] = 1
        cv2.imshow("A", binary)
        cv2.waitKey(1)
        binary = cv2.bitwise_and(binary, cv2.bitwise_not(canvas))
        print(canvas_count)
        new_canvas_count = np.count_nonzero(canvas==1)
        if new_canvas_count != canvas_count:
            canvas_count = new_canvas_count
            point_deleted = True
    print("Finished")
    return binary
if __name__ == "__main__":
    pathMask = cv2.imread("X:/pathMask1.png", 0)
    pathMask = cv2.imread("X:/thin_test.png", 0)
    _, pathMask = cv2.threshold(pathMask,127,255,cv2.THRESH_BINARY)
    import imutils
    # pathMask = imutils.resize(pathMask, height=200)
    thin = gouHall(pathMask)
    cv2.imshow("A", thin)
    cv2.waitKey(0)