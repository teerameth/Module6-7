import cv2
import numpy as np
import math
import imutils
from matplotlib import pyplot as plt
colors = [(255, 255, 255), (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

def imsave(img, filename):
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.savefig('./img/output/' + filename + '.png', dpi=300)
def imshow(img):
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()
def imshows(imgs, labels):
    fig, axes = plt.subplots(nrows=1, ncols=len(imgs), figsize=(8, 4), sharex=True, sharey=True)
    ax = axes.ravel()
    for i in range(len(imgs)):
        ax[i].imshow(imgs[i], cmap=plt.cm.gray)
        ax[i].axis('off')
        ax[i].set_title(labels[i], fontsize=20)
    fig.tight_layout()
    plt.show()
def imdraw(c, frame, mode = 0, color = 0, size = 1, show = True):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    cv2.drawContours(canvas, [c], 0, colors[color], size)
    if show: imshow(canvas)
def imdraws(cnts, frame, mode = 0, color = 0, size = 1, show = True):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    cv2.drawContours(canvas, cnts, -1, colors[color], size)
    if show: imshow(canvas)
def imdraws_color(cnts, frame, mode = 0, size = 1, show = True):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    for i in range(len(cnts)):
        cv2.drawContours(canvas, cnts, i, colors[i%len(colors)], size)
        if mode == 2: # Object by Object
            if show: imshow(canvas)
            canvas = np.zeros_like(frame, np.uint8)
    if mode != 2 and show: imshow(canvas)
    return canvas
def immask(c, frame):
    mask = np.ones(frame.shape[:2], dtype="uint8") * 255
    cv2.drawContours(mask, [c], 0, 0, -1)
    return mask
def immasks(cnts, frame, show = True):
    mask = np.ones(frame.shape[:2], dtype="uint8") * 255
    cv2.drawContours(mask, cnts, -1, 0, -1)
    if show: imshow(mask)
    return mask
def implot(c, frame, mode = 0, size = 1):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    for i in range(len(c)):
        center = (c[i][0][0], c[i][0][1])
        cv2.circle(canvas, center, size, (255, 255, 255), -1)
        cv2.imshow("Preview Plot", canvas)
        cv2.waitKey(1)
    return canvas