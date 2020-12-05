import cv2
import numpy as np
import math
import random

def run(use_random=True, visualize=False):
    cuda_stream = cv2.cuda_Stream()
    backSub = cv2.cuda.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)
    # backSub = cv2.createBackgroundSubtractorKNN(history=30, dist2Threshold=400.0, detectShadows=True)
    warped_list = np.load("X:/warped.npy", mmap_mode='r')
    valid_mask_list = np.load("X:/mask.npy", mmap_mode='r')
    i = 0
    frame_counter = 0
    mode = True
    N = 1000
    bg_list = []
    for frame_counter in range(N):
        if use_random: index = random.randint(0, len(warped_list) - 1)
        else: index = frame_counter % len(warped_list)
        warped = warped_list[index]
        valid_mask = valid_mask_list[index]
        frame_counter += 1
        if frame_counter > 500 and random.randint(0, N - frame_counter) < 200:
            warped_final = bg_list[random.randint(0, len(bg_list) - 1)]
        else:
            if frame_counter < 200:
                mean_canvas = np.ones(warped.shape, dtype="uint8") * 200
                mean_canvas = cv2.bitwise_or(mean_canvas, mean_canvas, mask=cv2.bitwise_not(valid_mask))
            else:
                mean_canvas = cv2.bitwise_or(bg, bg, mask=cv2.bitwise_not(valid_mask))
                if frame_counter % 8 == 0: bg_list.append(bg)
            valid_mask = cv2.bitwise_or(warped, warped, mask=valid_mask)
            final = cv2.bitwise_or(mean_canvas, valid_mask)
            if visualize: cv2.imshow('Passed', final)
            final_gpu = cv2.cuda_GpuMat()
            final_gpu.upload(final)
            final = cv2.cuda_GpuMat(final)
            fgMask = backSub.apply(final, -1, cuda_stream)
            fgMask = fgMask.download()
            # fgMask = cv2.morphologyEx(fgMask, cv2.MORPH_CLOSE, kernel=np.ones((5,5),np.uint8))
            # cv2.imshow('FG Mask', fgMask)
            bg = cv2.cuda_GpuMat(final_gpu.size(), final_gpu.type())
            backSub.getBackgroundImage(cuda_stream, bg)
            bg = bg.download()
            bg_list.append(bg)
            if visualize:
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
    # cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run(visualize=True)