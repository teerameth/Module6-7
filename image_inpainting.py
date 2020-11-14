import cv2

radius = 10

image = cv2.imread("1.jpg")
mask = cv2.imread("1_mask.jpg", 0)
cv2.imshow("Image", image)
cv2.imshow("Mask", mask)
output = cv2.inpaint(image, mask, radius, flags=cv2.INPAINT_TELEA) # inpainting method [cv2.INPAINT_TELEA, cv2.INPAINT_NS]
cv2.imshow("Output", output)
cv2.waitKey(0)