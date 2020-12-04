print(__doc__)
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import cv2
import random
import imutils

def cluster(frame, visualize = False, esp_value=0.4, min_samples=500, N=1000, scale = 4):
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
            canvas = np.zeros((800, 800, 3))
            for i in range(N):
                if labels[i] == -1: cv2.circle(canvas, index_list[i], 2, (0, 0, 255), -1)
                if labels[i] == 0:
                    cv2.circle(canvas, index_list[i], 2, (0, 255, 0), -1)
                else: cv2.circle(canvas, index_list[i], 2, (255, 0, 0), -1)
            cv2.imshow("A", canvas)
            cv2.imshow("B", mask)
            cv2.waitKey(1)

    # if visualize:
    #     # Number of clusters in labels, ignoring noise if present.
    #     n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    #     n_noise_ = list(labels).count(-1)
    #     print('Estimated number of clusters: %d' % n_clusters_)
    #     print('Estimated number of noise points: %d' % n_noise_)
    #     # print("Silhouette Coefficient: %0.3f" % metrics.silhouette_score(X, labels))
    #     # #############################################################################
    #     # Plot result
    #     import matplotlib.pyplot as plt
    #     # Black removed and is used for noise instead.
    #     unique_labels = set(labels)
    #     colors = [plt.cm.Spectral(each)
    #               for each in np.linspace(0, 1, len(unique_labels))]
    #     for k, col in zip(unique_labels, colors):
    #         if k == -1:
    #             # Black used for noise.
    #             col = [0, 0, 0, 1]
    #         class_member_mask = (labels == k)
    #         xy = X[class_member_mask & core_samples_mask]
    #         plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
    #                  markeredgecolor='k', markersize=14)
    #         xy = X[class_member_mask & ~core_samples_mask]
    #         plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
    #                  markeredgecolor='k', markersize=6)
    #     plt.title('Estimated number of clusters: %d' % n_clusters_)
    #     plt.show()
    if scale !=1 : mask = imutils.resize(mask, height=original_height)
    return mask

if __name__ == "__main__":
    import time
    path = "X:/final/"
    file_names = ["BGsub Serial Frank.png", "Median with valid mask.png", "BGsub Serial.png", "BGsub Stochastic Frank.png", "BGsub Stochastic.png"]
    for N_value in np.arange(500, 2000, 500):
        for esp_value in np.arange(0.1, 1.0, 0.1):
            for file_name in file_names:
                file_path = path + file_name
                frame = cv2.imread(file_path)
                start = time.time()
                mask = cluster(frame, esp_value=0.7,  scale=2, N=N_value)
                t = str(int(time.time()-start))
                print("Time usage: " + t + "sec.")
                cv2.imwrite(path + "mask/" + file_name[:-4] +
                            "_esp_" + str(esp_value)[:4].replace('.', '_') +
                            "_time_" + t +
                            "_N_" + str(N_value) + ".png", mask)
                cv2.imshow("Mask", mask)
                cv2.waitKey(1)