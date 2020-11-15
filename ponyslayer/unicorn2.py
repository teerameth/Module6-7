import cv2
import numpy as np
import SpoutSDK
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

class Camera():
    def __init__(self):
        self.cap = cv2.VideoCapture(cv2.CAP_DSHOW)
        codec = 0x47504A4D  # MJPG
        self.cap.set(cv2.CAP_PROP_FPS, 30.0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(3, 1920)
        self.cap.set(4, 1080)
    def read(self):
        return self.cap.read()[1]

def main():
    #Test all funnction
    pass
if __name__ == '__main__':
    main()