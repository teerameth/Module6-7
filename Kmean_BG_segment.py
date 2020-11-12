import cv2
import numpy as np
import matplotlib.pyplot as plt

def segment(img, K=3):
    K = 3
    attempts=10

    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    H, S, V = cv2.split(img)

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
    result_image = res.reshape((img.shape[:2]))
    mask = np.array(result_image, dtype="uint8")
    return 255 - mask
    # figure_size = 15
    # plt.figure(figsize=(figure_size,figure_size))
    # plt.subplot(1,2,1),plt.imshow(img)
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(1,2,2),plt.imshow(result_image)
    # plt.title('Segmented Image when K = %i' % K), plt.xticks([]), plt.yticks([])
    # plt.show()

def main():
    img = cv2.imread("Real6.png")
    mask = segment(img, K=3)
    cv2.imshow("A", mask)
    cv2.waitKey(0)

if __name__ == '__main__':
    main()