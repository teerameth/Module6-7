import cv2
import numpy as np
import urllib.request
import retinex
import json

with open('config.json', 'r') as f:
    config = json.load(f)

backSub = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=16, detectShadows=True)

stream = urllib.request.urlopen('http://10.61.1.41:5280/')
bytes = b''
i=0
j=193001

while True:
    try:
        bytes += stream.read(1024)
        a = bytes.find(b'\xff\xd8') # JPEG start
        b = bytes.find(b'\xff\xd9') # JPEG end
        if a!=-1 and b!=-1:
            jpg = bytes[a:b+2] # actual image
            bytes= bytes[b+2:] # other informations

            # decode to colored image ( another option is cv2.IMREAD_GRAYSCALE )
            img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
            img = cv2.rotate(img, cv2.ROTATE_180)
            cv2.imshow('Window name',img) # display image while receiving data

            # img_msrcr = retinex.MSRCR(
            # img,
            # config['sigma_list'],
            # config['G'],
            # config['b'],
            # config['alpha'],
            # config['beta'],
            # config['low_clip'],
            # config['high_clip']
            # )
            # img_amsrcr = retinex.automatedMSRCR(
            #     img,
            #     config['sigma_list']
            # )
            # img_msrcp = retinex.MSRCP(
            #     img,
            #     config['sigma_list'],
            #     config['low_clip'],
            #     config['high_clip']        
            # )
            # shape = img.shape
            # cv2.imshow('retinex', img_msrcr)
            # cv2.imshow('Automated retinex', img_amsrcr)
            # cv2.imshow('MSRCP', img_msrcp)

            fgMask = backSub.apply(cv2.GaussianBlur(img,(5,5),0))
            bg = backSub.getBackgroundImage()
            cv2.imshow('BG', bg)
            equ = cv2.equalizeHist(cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY))
            cv2.imshow('Hist Equ', equ)
            if j%1000==0:
                cv2.imwrite(str(int(j/1000)) + ".jpg", equ)
            j += 1

            key = cv2.waitKey(1)
            if  key==27: # if user hit esc
                exit(0) # exit program
            # elif key==ord(' '):
            #     cv2.imwrite(str(i) + ".jpg", img)
            #     i += 1
    except : continue
out.release()