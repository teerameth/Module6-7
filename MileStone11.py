# -*- coding: utf-8 -*-
import pyqtgraph as pg
import pyqtgraph.ptime as ptime
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import cv2
import numpy as np
import imutils
import v4l2capture
import select
from pyqtgraph.dockarea import *

### Camera init ###
width, height = 1920, 1080
cap = v4l2capture.Video_device("/dev/video2")
size_x, size_y = cap.set_format(width, height, fourcc='MJPG')
cap.create_buffers(1)
cap.queue_all_buffers()
cap.start()
### UI init ###
app = QtGui.QApplication([])
win = QtGui.QMainWindow()
area = DockArea()
win.setCentralWidget(area)
win.resize(1920,1080)
win.setWindowTitle('Pony Slayer: Mile Stone I')

### Create Dock ###
d1 = Dock("Dock1 - Preview", size=(960, 540))
d2 = Dock("Dock2 - Transformed", size=(540,540))
d3 = Dock("Dock3 - Control Panel", size=(420,540))
d4 = Dock("Dock4 - 3D Preview", size=(960,540))
d5 = Dock("Dock5 - Extract Marker", size=(540,540))
area.addDock(d1, 'left')      
area.addDock(d2, 'right', d1) 
area.addDock(d3, 'right', d2)
area.addDock(d4, 'bottom', d1)
area.addDock(d5, 'bottom', d2)
### Add widgets into each Dock ###
## Dock 1 - Preview image from camera & overlay ##
w1 = pg.GraphicsLayoutWidget()
d1.addWidget(w1)
view = w1.addViewBox()
view.setAspectLocked(True)
view.setMouseEnabled(x=False, y=False) # Make it unable to move by mouse
preview_img_rgb = pg.ImageItem(border='w') # Preview Image
view.addItem(preview_img_rgb)
## Dock 1.5 - Transformed image from ROI
# v2a = w1.addViewBox(row=1, col=0, lockAspect=True)
ROI = pg.PolyLineROI([[100,100], [500,100], [500, 500], [100,500]], closed=True) # Count from buttom left
def update_roi(roi): # update transformed image from ROI
    pass
ROI.sigRegionChanged.connect(update_roi)
view.addItem(ROI)
update_roi(ROI)
# print(ROI.getArrayRegion(preview_img_rgb, img1a), levels=(0, arr.max()))

updateTime = ptime.time()
fps = 0
def updatew1(): # Update preview image (and get image from camera)
    global preview_img_rgb, updateTime, fps
    select.select((cap,), (), ())
    image_data = cap.read_and_queue()
    frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_90_CLOCKWISE)
    ## Display the data
    preview_img_rgb.setImage(frame_rgb)
    QtCore.QTimer.singleShot(1, updatew1)
    now = ptime.time()
    fps2 = 1.0 / (now-updateTime)
    updateTime = now
    fps = fps * 0.9 + fps2 * 0.1
    print("%0.1f fps" % fps)


## first dock gets save/restore buttons
# w1 = pg.LayoutWidget()
# label = QtGui.QLabel(""" -- Mile Stone I -- """)
# saveBtn = QtGui.QPushButton('Save dock state')
# restoreBtn = QtGui.QPushButton('Restore dock state')
# restoreBtn.setEnabled(False)
# w1.addWidget(label, row=0, col=0)
# w1.addWidget(saveBtn, row=1, col=0)
# w1.addWidget(restoreBtn, row=2, col=0)
# d1.addWidget(w1)
# state = None
# def save():
#     global state
#     state = area.saveState()
#     restoreBtn.setEnabled(True)
# def load():
#     global state
#     area.restoreState(state)
# saveBtn.clicked.connect(save)
# restoreBtn.clicked.connect(load)


# w2 = pg.console.ConsoleWidget()
# d2.addWidget(w2)

# ## Hide title bar on dock 3
# d3.hideTitleBar()
# w3 = pg.PlotWidget(title="Plot inside dock with no title bar")
# w3.plot(np.random.normal(size=100))
# d3.addWidget(w3)

# w4 = pg.PlotWidget(title="Dock 4 plot")
# w4.plot(np.random.normal(size=100))
# d4.addWidget(w4)

# w5 = pg.ImageView()
# w5.setImage(np.random.normal(size=(100,100)))
# d5.addWidget(w5)

# w6 = pg.PlotWidget(title="Dock 6 plot")
# w6.plot(np.random.normal(size=100))
# d6.addWidget(w6)



win.show()
updatew1()


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
