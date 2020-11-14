import cv2
img = cv2.imread("../img/Real5.png")
res = cv2.ximgproc.edgePreservingFilter(img, 9, 20)
cv2.imshow("Original", img)
cv2.imshow("Res", res)
cv2.waitKey(0)
cv2.imwrite("Real5_1.png", res)