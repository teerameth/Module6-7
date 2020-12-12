print(__doc__)
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import cv2
import random
import imutils
def cluster_grap(frame, visualize = False, esp_value=0.7, min_samples=500, N=1000):
    original_height = frame.shape[0]
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(HSV)
    X_base = []
    index_list_base = []
    for i in range(N):
        y = random.randrange(0, H.shape[0] - 1)
        x = random.randrange(0, H.shape[1] - 1)
        point = (x, y)
        X_base.append((H[point], S[point], V[point]))
        index_list_base.append(point)

    X = StandardScaler().fit_transform(X_base)

    # Compute DBSCAN
    db = DBSCAN(eps=esp_value, min_samples=min_samples).fit(X)  # esp=0.3
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    ### For grabcut ###
    mask = np.ones(frame.shape[:2], np.uint8) * 2
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)

    canvas = np.zeros((H.shape[0], H.shape[1], 3)) # Visualize canvas
    for i in range(len(index_list_base)):
        point = (index_list_base[i][1], index_list_base[i][0])
        if labels[i] == -1:
            cv2.circle(canvas, point, 2, (0, 0, 255), -1)
            cv2.circle(mask, point, 2, 0, -1)
        if labels[i] == 0:
            cv2.circle(canvas, point, 2, (0, 255, 0), -1)
            cv2.circle(mask, point, 2, 1, -1)
        else:
            cv2.circle(canvas, point, 2, (255, 0, 0), -1)

    mask, bgdModel, fgdModel = cv2.grabCut(frame, mask, None, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_MASK)
    mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    if visualize:
        cv2.imshow("src", frame)
        cv2.imshow("Clustered Sample", canvas)
        cv2.imshow("Maks", mask2 * 255)
        cv2.waitKey(0)
    mask = np.asarray(mask2*255, dtype=np.uint8)
    ## Remove outer border ##
    (h, w) = mask.shape
    mask_border = cv2.rectangle(np.ones_like(mask)*255, (int(0.1 * w), int(0.1 * h)),
                                (int(0.9 * w), int(0.9 * h)), 0, -1)
    return cv2.bitwise_or(mask_border, mask)

def cluster(frame, visualize = False, esp_value=0.7, min_samples=500, N=1000, scale = 2):
    original_height = frame.shape[0]
    if scale != 1: frame = imutils.resize(frame, height=int(original_height/scale))
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(HSV)
    mask = np.zeros(H.shape)
    X_base = []
    index_list_base = []
    for i in range(N):
        y = random.randrange(0, H.shape[0] - 1)
        x = random.randrange(0, H.shape[1] - 1)
        point = (x, y)
        X_base.append((H[point], S[point], V[point]))
        if visualize: index_list_base.append(point)
    for y_real in range(H.shape[0]):
        X = []
        index_list = []
        for x_real in range(H.shape[1]): # Real data that we want
            point = (x_real, y_real)
            X.append((H[point], S[point], V[point]))
            if visualize: index_list.append(point)
        X += X_base

        X = StandardScaler().fit_transform(X)
        # #############################################################################
        # Compute DBSCAN
        db = DBSCAN(eps=esp_value, min_samples=min_samples).fit(X) # esp=0.3
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_
        # print(labels)

        lable_list, counts = np.unique(labels, return_counts=True) # [-1, 0, 1, 2 ,3, ...] (-1 for None class)
        max_index = np.argmax(counts)
        # print(max_index)

        for x_real in range(H.shape[1]):  # Real data that we want
            if labels[x_real] == 0: mask[(x_real, y_real)] = 255

        # Visualize
        if visualize:
            canvas = np.zeros((H.shape[0], H.shape[1], 3))
            for i in range(len(index_list_base)):
                if labels[i+H.shape[0]] == -1: cv2.circle(canvas, index_list_base[i], 2, (0, 0, 255), -1)
                if labels[i+H.shape[0]] == 0: cv2.circle(canvas, index_list_base[i], 2, (0, 255, 0), -1)
                else: cv2.circle(canvas, index_list_base[i], 2, (255, 0, 0), -1)
            cv2.imshow("A", canvas)
            cv2.imshow("B", mask)
            cv2.waitKey(1)

    border_mask = np.ones_like(mask)*255
    ratio = 0.05
    cv2.rectangle(border_mask, (int(H.shape[0]*ratio), int(H.shape[1]*ratio)), (int(H.shape[0]*(1-ratio)), int(H.shape[0]*(1-ratio))), 0, -1)
    mask = cv2.bitwise_or(mask, border_mask)
    if scale !=1 : mask = imutils.resize(mask, height=original_height)
    # mask = np.asarray(mask, dtype=np.uint8)
    # _, thresh = cv2.threshold(cv2.cvtColor(imutils.resize(frame, height=800), cv2.COLOR_BGR2GRAY), 80, 255, cv2.THRESH_BINARY_INV)
    # mask = cv2.bitwise_or(mask, thresh)
    return np.asarray(mask, dtype=np.uint8)

if __name__ == "__main__":
    import time
    frame = cv2.imread("X:/final.png")
    # frame = cv2.imread("../img/Real7.png")
    start = time.time()
    # mask = cluster(frame, esp_value=0.7,  scale=2, N=1000, visualize=True)
    mask = cluster_grap(frame, esp_value=0.7, N=1000, visualize=True)
    t = str(int(time.time()-start))
    print("Time usage: " + t + "sec.")
    cv2.imshow("Mask", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()