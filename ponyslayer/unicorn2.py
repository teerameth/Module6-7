import cv2
import numpy as np
import SpoutSDK
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

class Camera():
    def __init__(self):
        self.cap = cv2.VideoCapture(cv2.CAP_DSHOW)
        codec = 0x47504A4D  # MJPG
        self.cap.set(cv2.CAP_PROP_FPS, 30.0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_SETTINGS, 1)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 2)
        self.cap.set(3, 1920)
        self.cap.set(4, 1080)
    def read(self):
        return self.cap.read()[1]

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
    #Test all funnction
    pass
if __name__ == '__main__':
    main()