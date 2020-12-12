import cv2
import imutils
import numpy as np
import glob
import frankenstein

def get_colormap(blur=35):
    filelist = glob.glob("X:/*")
    frame = None
    pathMask_list = []
    for filepath in filelist:
        if 'pathMask' in filepath: pathMask_list.append(cv2.imread(filepath, 0))
        if 'final' in filepath: frame = cv2.imread(filepath)
    canvas = np.zeros_like(frame)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.medianBlur(frame, blur)
    for pathMask in pathMask_list:
        pathMask_colored = cv2.cvtColor(pathMask, cv2.COLOR_GRAY2BGR)
        masked = cv2.bitwise_and(pathMask, frame)
        result = cv2.equalizeHist(masked)
        colorMap = cv2.applyColorMap(result, cv2.COLORMAP_JET)
        canvas = cv2.bitwise_or(canvas, cv2.bitwise_and(pathMask_colored, colorMap))
    return np.asarray(canvas, np.uint8)
def get_colormap_frank():
    filelist = glob.glob("X:/*")
    frankenstein.run(50)
    frank_list = np.load("X:/frank.npy")
    pathMask_list = []
    colorMap_list = []
    for filepath in filelist:
        if 'pathMask' in filepath: pathMask_list.append(cv2.imread(filepath, 0))
        if 'final' in filepath: frame = cv2.imread(filepath)
    for frank in frank_list:
        canvas = np.zeros_like(frank)
        frame = cv2.cvtColor(frank, cv2.COLOR_BGR2GRAY)
        for pathMask in pathMask_list:
            pathMask_colored = cv2.cvtColor(pathMask, cv2.COLOR_GRAY2BGR)
            masked = cv2.bitwise_and(pathMask, frame)
            result = cv2.equalizeHist(masked)
            colorMap = cv2.applyColorMap(result, cv2.COLORMAP_JET)
            canvas = cv2.bitwise_or(canvas, cv2.bitwise_and(pathMask_colored, colorMap))
        colorMap_list.append(canvas)
        cv2.imshow("A", canvas)
        cv2.waitKey(100)
    cv2.imshow("A", np.asarray(np.median(np.dstack(colorMap_list), axis=2), dtype=np.uint8))
    cv2.waitKey(0)
    # return np.array(canvas, np.uint8)
def demo():
    # colorMap = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
    identity = np.arange(256, dtype = np.dtype('uint8'))
    serial = np.arange(1, 256, dtype=np.dtype('uint8'))
    zeros = np.zeros(256, np.dtype('uint8'))
    lut = np.dstack((identity, identity, zeros))
    frame[:,:,2] = 0 # zeroing red channel
    # colorMap = cv2.LUT(frame, lut)
    b_max = 20
    frame[frame[:,:,0] > b_max, 0] = b_max;
    cv2.imshow("Color Map", frame)
    cv2.waitKey(0)
if __name__ == "__main__":
    # get_colormap_frank()
    colorMap = get_colormap()
    cv2.imshow("A", colorMap)
    cv2.waitKey(0)