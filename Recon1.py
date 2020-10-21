import cv2
import glob
import numpy as np
from matplotlib import pyplot as plt
def imshow(img):
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()
img_src = "C:/Users/teera/Documents/Unity Projects/module/Assets/img/" # Path to folder contain scanned img
paths = glob.glob(img_src + "*.png")
scan_cnt = 5
crop_size = 290
img_size_x = 2000
img_size_y = 2000
img_list = []
for path in paths:
    img = {"index":int(path[len(img_src):-4]), "path":path, "src":cv2.imread(path)[int((img_size_y - crop_size)/2):int((img_size_y + crop_size)/2), int((img_size_y - crop_size)/2):int((img_size_y + crop_size)/2)]}
    img_list.append(img)
#     imshow(img["src"])
img_list = sorted(img_list, key=lambda x : x.get("index"))

v_buff = []
for i in range(scan_cnt):
    v_buff.append(np.vstack([img_list[v]["src"] for v in range(scan_cnt*i+4, scan_cnt*i-1, -1)]))
cv2.imshow("Preview", np.hstack(v_buff))
# cv2.imwrite("reconstructed.jpg", np.hstack(v_buff))