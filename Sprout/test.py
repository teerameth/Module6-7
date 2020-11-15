import cv2
import numpy as np
# Add relative directory ../Library to import path, so we can import the SpoutSDK.pyd library. Feel free to remove these if you put the SpoutSDK.pyd file in the same directory as the python scripts.
# import sys
# sys.path.append('C:/Users/teera/Documents/GitHub/Module6-7/Sprout/')
import numpy as np
import SpoutSDK
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
width, height = 1280, 720
spoutSenderWidth, spoutSenderHeight = 1280, 720

cap = cv2.VideoCapture(cv2.CAP_DSHOW)
codec = 0x47504A4D  # MJPG
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(3, width)
cap.set(4, height)

def main():
    # window details
    display = (width, height)
    # window setup
    pygame.init()
    pygame.display.set_caption('Spout Python Sender')
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.gl_set_attribute(pygame.GL_ALPHA_SIZE, 8)

    # OpenGL init
    # setup OpenGL perspective
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    # setup default colours, blending modes, rotation and translation parameters
    glMatrixMode(GL_MODELVIEW)
    # reset the drawing perspective
    glLoadIdentity()
    # can disable depth buffer because we aren't dealing with multiple geometry in our scene
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_ALPHA_TEST)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glTranslatef(0, 0, -5)
    glRotatef(25, 2, 1, 0)

    # init spout sender
    spoutSender = SpoutSDK.SpoutSender()
    # Its signature in c++ looks like this: bool CreateSender(const char *Sendername, unsigned int width, unsigned int height, DWORD dwFormat = 0);
    spoutSender.CreateSender('Spout Python Sender', spoutSenderWidth, spoutSenderHeight, 0)

    # init spout sender texture ID
    senderTextureID = glGenTextures(1)

    # initialise texture
    glBindTexture(GL_TEXTURE_2D, senderTextureID)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    # fill texture with blank data
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, spoutSenderWidth, spoutSenderHeight, 0);
    glBindTexture(GL_TEXTURE_2D, 0)

    while True:
        frame = cap.read()[1]
        # Copy the frame from the opencv into the sender texture
        glBindTexture(GL_TEXTURE_2D, senderTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, spoutSenderWidth, spoutSenderHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, cv2.flip(frame, 0))

        # send texture to Spout
        # Its signature in C++ looks like this: bool SendTexture(GLuint TextureID, GLuint TextureTarget, unsigned int width, unsigned int height, bool bInvert=true, GLuint HostFBO = 0);
        spoutSender.SendTexture(np.int(senderTextureID), GL_TEXTURE_2D, spoutSenderWidth, spoutSenderHeight, True, 0)

        # update display
        pygame.display.flip()

        pygame.time.wait(10)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
if __name__ == '__main__':
    main()