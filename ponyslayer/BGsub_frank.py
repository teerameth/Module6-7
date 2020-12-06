import cv2
import numpy as np
import math
import random

def run(use_random=True, visualize=False):
    cuda_stream = cv2.cuda_Stream()
    backSub = cv2.cuda.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)
    # backSub = cv2.createBackgroundSubtractorKNN(history=30, dist2Threshold=400.0, detectShadows=True)
    warped_list = np.load("X:/frank.npy", mmap_mode='r')
    N = 1000
    bg_list = []
    for frame_counter in range(N):
        if use_random: index = random.randint(0, len(warped_list) - 1)
        else: index = frame_counter % len(warped_list)
        warped = warped_list[index]
        warped_gpu = cv2.cuda_GpuMat()
        warped_gpu.upload(warped)
        fgMask = backSub.apply(warped_gpu, -1, cuda_stream)
        fgMask = fgMask.download()
        fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_CLOSE, kernel=np.ones((5,5),np.uint8))
        if visualize: cv2.imshow('FG Mask', fgMask)
        bg = cv2.cuda_GpuMat(warped_gpu.size(), warped_gpu.type())
        backSub.getBackgroundImage(cuda_stream, bg)
        bg = bg.download()
        if visualize: cv2.imshow('BG', bg)
        key = cv2.waitKey(1)
        if key == 27:
            break
    cv2.imwrite("X:/final.png", bg)
    if visualize: cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()