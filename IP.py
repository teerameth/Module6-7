import cv2
import numpy as np
import urllib.request

stream = urllib.request.urlopen('http://10.61.1.41:5280/')
bytes = b''
i=0
while True:
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8') # JPEG start
    b = bytes.find(b'\xff\xd9') # JPEG end
    if a!=-1 and b!=-1:
        jpg = bytes[a:b+2] # actual image
        bytes= bytes[b+2:] # other informations

        # decode to colored image ( another option is cv2.IMREAD_GRAYSCALE )
        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
        cv2.imshow('Window name',img) # display image while receiving data
        equ = cv2.equalizeHist(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        cv2.imshow('Hist Equ', equ)
        # print(img.shape)
        key = cv2.waitKey(1)
        if  key==27: # if user hit esc
            exit(0) # exit program
        elif key==ord(' '):
            cv2.imwrite(str(i) + ".jpg", img)
            i += 1

cap = cv2.VideoCapture("http://192.168.43.239/")

while True:
    _, img = cap.read()

    cv2.imshow("Perview", img)
    cv2.waitKey(1)