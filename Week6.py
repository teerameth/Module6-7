import cv2
import numpy as np
image = cv2.imread("task51.jpg")
image = image * 16
cv2.imshow("A", image)
cv2.waitKey(0)
cv2.imwrite("aaa.jpg", image)

#
# def nothing(x):
#     pass
#
# # Create a black image, a window
# img = np.zeros((300,512,3), np.uint8)
# cv2.namedWindow('image')
# image = cv2.imread("task51.jpg")
# # create trackbars for color change
# cv2.createTrackbar('R','image',0,255,nothing)
# cv2.createTrackbar('G','image',0,255,nothing)
# cv2.createTrackbar('B','image',0,255,nothing)
#
# while(1):
#     cv2.imshow('image',image)
#     k = cv2.waitKey(1) & 0xFF
#     if k == 27:
#         break
#
#     # get current positions of four trackbars
#     r = cv2.getTrackbarPos('R','image')
#     g = cv2.getTrackbarPos('G','image')
#     b = cv2.getTrackbarPos('B','image')
#
#     image[:] = [b,g,r]
#
# cv2.destroyAllWindows()
import cv2
import numpy as np
img = cv2.imread('task51.jpg',0)
equ = cv2.equalizeHist(img)
res = np.hstack((img,equ)) #stacking images side-by-side
cv2.imshow("A", res)
cv2.waitKey(0)