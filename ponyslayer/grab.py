import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('final.png')
mask = np.zeros(img.shape[:2],np.uint8)
bgdModel = np.zeros((1,65),np.float64)
fgdModel = np.zeros((1,65),np.float64)

drawingL = False # true if mouse is pressed
drawingR = False
mode = False # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawingL, drawingR,mode
    if event == cv2.EVENT_LBUTTONDOWN:
        drawingL = True
        ix,iy = x,y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawingL == True:
            cv2.circle(mask,(x,y),5,1,-1)
    elif event == cv2.EVENT_LBUTTONUP:
        drawingL = False
        cv2.circle(mask,(x,y),5,1,-1)
    if event == cv2.EVENT_RBUTTONDOWN:
        drawingR = True
        ix,iy = x,y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawingR == True:
            cv2.circle(mask,(x,y),5,0,-1)
    elif event == cv2.EVENT_RBUTTONUP:
        drawingR = False
        cv2.circle(mask,(x,y),5,0,-1)
mask = np.ones(img.shape[:2], np.uint8)*2
cv2.namedWindow('image')
cv2.imshow("image", img)
cv2.setMouseCallback('image',draw_circle)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == 27:
        break
cv2.destroyAllWindows()
mask, bgdModel, fgdModel = cv2.grabCut(img,mask,None,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)
mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
cv2.imshow("A", mask2*255)
cv2.waitKey(0)