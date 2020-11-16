import cv2
import numpy as np
import SpoutSDK
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
class VDO2Unity():
    def __init__(self):
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB)
        glutInitWindowSize(200, 200)
        glutCreateWindow('OpenGL Dummy')
        glutHideWindow()
        self.bufferCnt = 0
        self.textureID, self.width, self.height, self.spoutSender, self.senderTextureID  = {}, {}, {}, {}, {}

    def createChannel(self, width, height, name):
        self.bufferCnt += 1
        self.width[name], self.height[name] = width, height
        self.spoutSender[name] = SpoutSDK.SpoutSender()  # init spout sender
        self.spoutSender[name].CreateSender(name, self.width[name], self.height[name], 0)  # Its signature in c++ looks like this: bool CreateSender(const char *Sendername, unsigned int width, unsigned int height, DWORD dwFormat = 0);
        self.senderTextureID[name] = glGenTextures(1)  # init spout sender texture ID
        try: self.senderTextureID[name] = self.senderTextureID[name][-1]
        except: pass
        # initialise texture
        glBindTexture(GL_TEXTURE_2D, self.senderTextureID[name])
    def send(self, name, frame):
        # Copy the frame from the opencv into the sender texture
        glBindTexture(GL_TEXTURE_2D, self.senderTextureID[name])
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.width[name], self.height[name], 0, GL_BGR, GL_UNSIGNED_BYTE, cv2.flip(frame, 0))
        self.spoutSender[name].SendTexture(np.int(self.senderTextureID[name]), GL_TEXTURE_2D, self.width[name], self.height[name], True, 0)
def main():
    cap = cv2.VideoCapture(cv2.CAP_DSHOW)
    codec = 0x47504A4D  # MJPG
    cap.set(cv2.CAP_PROP_FPS, 30.0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(3, 1280)
    cap.set(4, 720)

    cast = VDO2Unity()
    cast.createChannel(1280, 720, '1')
    cast.createChannel(1280, 720, '2')
    cast.createChannel(1280, 720, '3')
    cast.createChannel(1280, 720, '4')
    cast.createChannel(1280, 720, '5')
    cast.createChannel(1280, 720, '6')
    cast.createChannel(1280, 720, '7')
    cast.createChannel(1280, 720, '8')

    while True:
        frame = cap.read()[1]
        cast.send('1', frame)
        cast.send('2', cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        cast.send('3', cv2.cvtColor(frame, cv2.COLOR_BGR2LAB))
        cast.send('4', cv2.cvtColor(frame, cv2.COLOR_BGR2HLS))
        frame = cv2.flip(frame, 0)
        cast.send('5', cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        cast.send('6', cv2.cvtColor(frame, cv2.COLOR_BGR2LAB))
        cast.send('7', cv2.cvtColor(frame, cv2.COLOR_BGR2HLS))
        cast.send('8', frame)

if __name__ == '__main__':
    main()