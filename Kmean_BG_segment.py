import cv2
import numpy as np
import matplotlib.pyplot as plt

def segment(img, K=3):
    attempts=10

    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower_gray = np.array([0, 5, 50], np.uint8)
    # upper_gray = np.array([179, 50, 255], np.uint8)
    # mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)
    # img_res = cv2.bitwise_and(img, img, mask = mask_gray)
    # cv2.imshow("A", img_res)
    # img_res = cv2.bitwise_and(img, img, mask = 255 - mask_gray)

    H, S, V = cv2.split(hsv)
    # vectorized = img.reshape((-1,3))
    vectorized = H.reshape((-1,1))
    vectorized = np.float32(vectorized)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

    ret,label,center=cv2.kmeans(vectorized,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS) #  cv.KMEANS_PP_CENTERS and cv.KMEANS_RANDOM_CENTERS.
    center = np.uint8(center)
    N = []
    for i in range(K): N.append(np.count_nonzero(label==i))
    max_index = N.index(max(N))
    label[label==max_index] = 1
    label[label!=max_index] = 0
    center[max_index] = 255
    res = center[label.flatten()]
    result_image = res.reshape((img.shape[:2])) # one channel
    # result_image = res.reshape((img.shape[:3])) # multi channel
    mask = np.array(result_image, dtype="uint8")
    # cv2.imshow("A", 255-mask)
    # cv2.waitKey(0)
    return 255 - mask
    # figure_size = 15
    # plt.figure(figsize=(figure_size,figure_size))
    # plt.subplot(1,2,1),plt.imshow(img)
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(1,2,2),plt.imshow(result_image)
    # plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
    # plt.show()

def main():
    img = cv2.imread("./img/Real6.png")
    mask = segment(img, K=3)
    # cv2.imshow("A", mask)
    # cv2.waitKey(0)

if __name__ == '__main__':
    main()