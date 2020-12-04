import numpy as np
import cv2
import random
def run(N=20):
    warped_list = np.load("X:/warped.npy", mmap_mode='r')
    valid_mask_list = np.load("X:/mask.npy", mmap_mode='r')
    n = 5
    num_pixel = 800*800
    frank_list = []
    for i in range(N):
        index = random.randint(0, len(warped_list) - 1)
        image = warped_list[index]
        mask = valid_mask_list[index]
        area = np.sum(mask == 255)
        while area/num_pixel*100 < 99:
            index = random.randint(0, len(warped_list) - 1)
            image_frank = warped_list[index]
            mask_frank = valid_mask_list[index]
            image_frank =  cv2.bitwise_or(image_frank, image_frank, mask=cv2.bitwise_not(mask))
            image = cv2.bitwise_or(image, image_frank)
            mask = cv2.bitwise_or(mask, mask_frank)
            area = np.sum(mask == 255)
        # print(int(area/num_pixel*100))
        frank_list.append(image)
        # cv2.imshow("A", image)
        # cv2.waitKey(1)
    np.save("X:/frank", frank_list)
if __name__ == "__main__":
    run(N=200)