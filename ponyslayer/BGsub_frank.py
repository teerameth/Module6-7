import cv2
import numpy as np
import math
import random


cuda_stream = cv2.cuda_Stream()
backSub = cv2.cuda.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)
# backSub = cv2.createBackgroundSubtractorKNN(history=30, dist2Threshold=400.0, detectShadows=True)

warped_list = np.load("X:/frank.npy", mmap_mode='r')

i=0
mode = True
N = 1000
bg_list = []
for frame_counter in range(N):
    index = random.randint(0, len(warped_list) - 1)
    warped = warped_list[index]

    warped_gpu = cv2.cuda_GpuMat()
    warped_gpu.upload(warped)
    fgMask = backSub.apply(warped_gpu, -1, cuda_stream)
    fgMask = fgMask.download()
    fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_CLOSE, kernel=np.ones((5,5),np.uint8))
    cv2.imshow('FG Mask', fgMask)
    bg = cv2.cuda_GpuMat(warped_gpu.size(), warped_gpu.type())
    backSub.getBackgroundImage(cuda_stream, bg)
    bg = bg.download()
    cv2.imshow('BG', bg)
    key = cv2.waitKey(1)
    if key == 27:
        break
    if key == ord('m'):
        mode = not mode
    if key == ord(' '):
        cv2.imwrite(str(i) + ".jpg", warped)
        i += 1
cv2.imwrite("X:/final.png", bg)
cv2.waitKey(0)
cv2.destroyAllWindows()