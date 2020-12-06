import cv2
import numpy as np
def kmean(img, K=3): # One channel only
    attempts = 10
    vectorized = img.reshape((-1, 1))
    vectorized = np.float32(vectorized)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret, label, center = cv2.kmeans(vectorized, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)  # cv.KMEANS_PP_CENTERS and cv.KMEANS_RANDOM_CENTERS.
    center = np.uint8(center)
    N = []
    for i in range(K): N.append(np.count_nonzero(label == i))
    max_index = N.index(max(N))
    label[label == max_index] = 1
    label[label != max_index] = 0
    center[max_index] = 255
    res = center[label.flatten()]
    result_image = res.reshape((img.shape[:2]))  # one channel
    # result_image = res.reshape((img.shape[:3])) # multi channel
    mask = np.array(result_image, dtype="uint8")
    # cv2.imshow("A", 255-mask)
    # cv2.waitKey(0)
    return 255 - mask