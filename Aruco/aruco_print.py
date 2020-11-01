import cv2
import numpy as np
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
X = 10
Y = 10
marker_size = 100 # pixels

marker_length = 0.04 # meter
gap = 0.01 # meter
bordersize = int(marker_size/(marker_length+gap)*gap/2) # pixels
image_size = (X*marker_size-2*bordersize, Y*marker_size-2*bordersize)

board = cv2.aruco.GridBoard_create(markersX=X, markersY=Y, markerLength=marker_length, markerSeparation=gap, dictionary=dictionary)
img = board.draw(outSize=image_size)
img = cv2.copyMakeBorder(
    img,
    top=bordersize,
    bottom=bordersize,
    left=bordersize,
    right=bordersize,
    borderType=cv2.BORDER_CONSTANT,
    value=[255, 255, 255]
)
# cv2.rectangle(img, (100, 100), (900, 900), (0, 0, 0), -1)
print(img.shape)
canvas11 = img[0:marker_size, 0:marker_size*5 ]
canvas12 = img[0:marker_size, marker_size*5 :-marker_size]
cv2.imshow("Canvas11", canvas11)
cv2.imshow("Canvas12", canvas12)
cv2.imshow("Marker Plane", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
canvas21 = img[0:marker_size*5, -marker_size:-1]
canvas22 = img[marker_size*5 :-marker_size,-marker_size:-1]
cv2.imshow("Canvas21", canvas21)
cv2.imshow("Canvas22", canvas22)
cv2.imshow("Marker Plane", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
canvas31 = img[-marker_size:-1, marker_size:marker_size*6]
canvas32 = img[-marker_size:-1, marker_size*6:-1]
cv2.imshow("Canvas31", canvas31)
cv2.imshow("Canvas32", canvas32)
cv2.imshow("Marker Plane", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
canvas41 = img[marker_size:marker_size*6, 0:marker_size]
canvas42 = img[marker_size*6:-1, 0:marker_size]
cv2.imshow("Canvas41", canvas41)
cv2.imshow("Canvas42", canvas42)
cv2.imshow("Marker Plane", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
imgs = [canvas11, canvas12, canvas21, canvas22, canvas31, canvas32, canvas41, canvas42]
i=0
for item in imgs:
    item = cv2.copyMakeBorder(
    item,
    top=1,
    bottom=1,
    left=1,
    right=1,
    borderType=cv2.BORDER_CONSTANT,
    value=[0, 0, 0])
    cv2.imwrite("./img/"+str(i)+".jpg", item)
    i+=1

