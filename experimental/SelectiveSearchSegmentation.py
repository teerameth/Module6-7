import cv2
cv2.setUseOptimized(True)
cv2.setNumThreads(12)
img = cv2.imread("../img/Real5.png")
ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()
ss.setBaseImage(img)
cv2.imshow("Source", img)
cv2.waitKey(1)
# ss.switchToSingleStrategy()
# ss.switchToSelectiveSearchFast()
ss.switchToSelectiveSearchQuality()
rects = ss.process()
nb_rects = 10
while True:
    wimg = img.copy()
    for i in range(len(rects)):
        if (i < nb_rects):
            x, y, w, h = rects[i]
            cv2.rectangle(wimg, (x, y), (x+w, y+h), (0, 255, 0), 1, cv2.LINE_AA)
    cv2.imshow("Output", wimg)
    c = cv2.waitKey()
    if (c == 100):
        nb_rects += 10
    elif (c == 97 and nb_rects > 10):
        nb_rects -= 10
    elif (c == 113):
        break
cv2.destroyAllWindows()
