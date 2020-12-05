import cv2
from skimage.measure import compare_ssim
import numpy as np
import random
import time
import threading, socket
def findMedian(images): return np.asarray(np.median(np.dstack(images), axis=2), dtype=np.uint8)
def apply_pair(imageA, maskA, imageB, maskB):
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
    (score, diff) = compare_ssim(grayA, grayB, full=True)
    diff = (diff * 255).astype("uint8")
    print("SSIM: {}".format(score))
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    ## Apply only valid area
    mask = cv2.bitwise_and(cv2.bitwise_and(thresh, maskA), maskB)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5),np.uint8))
    return mask
def apply_three(imageA, maskA, imageB, maskB, imageC, maskC):
    AB = apply_pair(imageA, maskA, imageB, maskB)
    AC = apply_pair(imageA, maskA, imageC, maskC)
    BC = apply_pair(imageB, maskB, imageC, maskC)
    A = cv2.bitwise_or(cv2.bitwise_and(AB, cv2.bitwise_not(BC)),  # A = AB & !BC
                       cv2.bitwise_and(AC, cv2.bitwise_not(BC)))  # A = AC & !BC
    B = cv2.bitwise_or(cv2.bitwise_and(AB, cv2.bitwise_not(AC)),  # B = AB & !AC
                       cv2.bitwise_and(AC, cv2.bitwise_not(AC)))  # B = BC & !AC
    C = cv2.bitwise_or(cv2.bitwise_and(AB, cv2.bitwise_not(AB)),  # C = AC & !AB
                       cv2.bitwise_and(AC, cv2.bitwise_not(AB)))  # C = BC & !AB
    return [A, B, C]
def apply_single_rand(imageA, maskA, pool, pool_valid, N):
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    mask_list = []
    for i in range(N):
        rand = random.randint(0, len(pool)-1)
        imageB = pool[rand]
        maskB = pool_valid[rand]
        grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
        (score, diff) = compare_ssim(grayA, grayB, full=True)
        diff = (diff * 255).astype("uint8")
        thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        ## Apply only valid area
        mask = cv2.bitwise_and(cv2.bitwise_and(thresh, maskA), maskB)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask_list.append(mask)
    mask_med = np.asarray(np.median(np.dstack(mask_list), axis=2), dtype=np.uint8)
    return cv2.bitwise_or(cv2.bitwise_not(maskA), mask_med)
def median_with_mask(images, masks):
    R, G, B = [], [] ,[]
    for img in images:
        r, g, b = cv2.split(img)
        R.append(r)
        G.append(g)
        B.append(b)
    mask_tmp = np.dstack(masks)
    image_tmp_r = np.dstack(R)
    image_tmp_g = np.dstack(G)
    image_tmp_b = np.dstack(B)
    # print(image_tmp_r.shape)
    # print(mask_tmp.shape)
    mask = np.sum(mask_tmp, axis=2, dtype=np.uint16) # 255 is masked, 0 is unmasked (we want to count)
    mask = np.asarray(len(images) - mask/255, dtype=np.uint8)
    # cv2.imshow("A", mask)
    # cv2.waitKey(0)
    # print(mask.shape)
    r = np.sort(image_tmp_r, axis=2) # Sort stacked pixel
    r = np.dsplit(r, len(images)) # Split stacked pixel as before (but sorted)
    g = np.dsplit(np.sort(image_tmp_g, axis=2), len(images))
    b = np.dsplit(np.sort(image_tmp_b, axis=2), len(images))
    # for i in range(len(images)):
    #     final = cv2.merge((r[i], g[i], b[i]))
    #     cv2.imshow("A", final)
    #     cv2.waitKey(0)
    result = np.zeros_like(images[0])
    # print(result.shape)
    # r = np.take(r, mask)
    # cv2.imshow("B", r)
    # cv2.waitKey(0)
    for row in range(result.shape[0]):
        for column in range(result.shape[1]):
            index = int((mask[row][column] + len(images))/2) # Median between max, mask_count
            if index >= len(images): index = len(images) - 1 # Pixel that never be seen will get index = len(images) and cause error indexing
            result[row][column][0] = r[index][row][column]
            result[row][column][1] = g[index][row][column]
            result[row][column][2] = b[index][row][column]
    # cv2.imshow("A", result)
    # cv2.waitKey(0)
    cv2.imwrite("X:/final.png", result)
    return result

def run(N=20): # Median with valid mask
    warped_list = np.load("X:/warped.npy", mmap_mode='r')
    valid_mask_list = np.load("X:/mask.npy", mmap_mode='r')
    warpeds = []
    valids = []
    for i in range(N):
        index = random.randrange(0, int(len(warped_list) / 3))
        warpeds.append(warped_list[index])
        valids.append(valid_mask_list[index])
    median_with_mask(warpeds, valids)

def run_rail(N=20, iteration=5): # Median with valid mask & Rail mask
    warped_list = np.load("X:/warped.npy", mmap_mode='r')
    valid_mask_list = np.load("X:/mask.npy", mmap_mode='r')
    warpeds = []
    valids = []
    masks = []
    for i in range(N):
        index = random.randrange(0, int(len(warped_list) / 3))
        warpeds.append(warped_list[index])
        valids.append(valid_mask_list[index])
        mask = apply_single_rand(warped_list[index], valid_mask_list[index], warped_list, valid_mask_list, iteration)
        # mask = cv2.GaussianBlur(mask,(5,5),0)
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Preview", warped_list[index])
        # cv2.waitKey(1)
        masks.append(mask)
    median_with_mask(warpeds, masks)

if __name__ == "__main__":
    run()
    # run_rail(N=20)