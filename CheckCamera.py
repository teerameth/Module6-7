import cv2 as cv 

def testDevice(source):
    cap = cv.VideoCapture(source) 
    if cap is None or not cap.isOpened(): print('Warning: unable to open video source: ', source)
    else: print(source, ' Avaliable')
    

for i in range(5):
    testDevice(i)