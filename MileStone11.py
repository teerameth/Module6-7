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

workspace_Xmax, workspace_Ymax = 400, 400 # Maximum workspace in millimeter

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

updateTime = ptime.time()
fps = 0
mousePoint = None

def update(): # Update preview image (and get image from camera)
    global preview_img_rgb, warped_img_rgb, updateTime, fps, ROI, mousePoint
    select.select((cap,), (), ())
    image_data = cap.read_and_queue()
    frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_90_CLOCKWISE)
    ## Display RAW image on Dock 1
    preview_img_rgb.setImage(frame_rgb, levels=(0, frame_rgb.max()))
    
    ## Get ROI position
    ROI_points_raw = ROI.getState()['points']
    ROI_points = []
    for point in ROI_points_raw:
        ROI_points.append([int(point[1]), int(point[0])]) # Convert Qt coordinate system to pixel coordinate system
    ## Transform from Image to World coordinate
    maxWidth, maxHeight = 540, 540
    dst_points = np.array([[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]], dtype = "float32")
    HM, status = cv2.findHomography(np.asarray(ROI_points), dst_points) # get HM(Homography Matrix)
    warped_img = cv2.warpPerspective(frame_rgb, HM, (maxWidth, maxHeight))
    ## Display Warped image on Dock 2
    warped_img = cv2.rotate(warped_img, cv2.ROTATE_90_CLOCKWISE)
    warped_img_rgb.setImage(warped_img)
    ## Update label of pixel coordinate of crosshair on Dock 2
    if mousePoint is not None:
        label_pixel.setText("<span style='font-size: 24pt'>Pixel coordinate: <span style='color: red'>x=%0.1f, y=%0.1f</span></span>" % (mousePoint.x(), mousePoint.y()))
        label_image.setText("<span style='font-size: 24pt'>Image coordinate: <span style='color: green'>x=%0.1f, y=%0.1f</span></span>" % ((mousePoint.x()-540/2)/540*workspace_Xmax, (mousePoint.y()-540/2)/540*workspace_Ymax))

    
    
    QtCore.QTimer.singleShot(1, update) # Update UI
    now = ptime.time()
    fps2 = 1.0 / (now-updateTime)
    updateTime = now
    fps = fps * 0.9 + fps2 * 0.1
    # print("%0.1f fps" % fps)

### Create Dock ###
d1 = Dock("Dock1 - Preview",        size=(960,540))
d2 = Dock("Dock2 - Transformed",    size=(540,540))
d3 = Dock("Dock3 - Control Panel",  size=(420,540))
d4 = Dock("Dock4 - 3D Preview",     size=(960,540))
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
view = w1.addViewBox(lockAspect=True)
view.setMouseEnabled(x=False, y=False) # Make it unable to move by mouse
# Get first frame #
select.select((cap,), (), ())
image_data = cap.read_and_queue()
frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
preview_img_rgb = pg.ImageItem(frame_rgb, border='w') # Preview first Image
view.addItem(preview_img_rgb)
# Add ROI as overlay #
ROI = pg.PolyLineROI([[100,100], [500,100], [500, 500], [100,500]], closed=True) # Default ROI (count from buttom left)
view.addItem(ROI)
## Dock 2 - Transformed image from ROI ##
w2 = pg.GraphicsLayoutWidget()
d2.addWidget(w2)
view2 = w2.addViewBox(lockAspect=True)
view2.invertY(True) # Fix inversed Y-axis
view2.setMouseEnabled(x=False, y=False) # Make it unable to move by mouse
# Display first transformed frame(dummy)
warped_img_rgb = pg.ImageItem(np.zeros((540,540,3), np.uint8), border='w')
view2.addItem(warped_img_rgb)

# Create cross hair (2 perspective infinite lines)
vLine = pg.InfiniteLine(angle=90, movable=False)
hLine = pg.InfiniteLine(angle=0, movable=False)
view2.addItem(vLine, ignoreBounds=True)
view2.addItem(hLine, ignoreBounds=True)
label_pixel = pg.LabelItem(justify='left')
label_image = pg.LabelItem(justify='left')
label_pixel.setPos(0, 0)
label_image.setPos(0, 30)
view2.addItem(label_pixel)
view2.addItem(label_image)


def mouseMoved(evt):
    global mousePoint # Update global mousePoint (wait for called by update())
    pos = evt[0]  ## using signal proxy turns original arguments into a tuple
    if view2.sceneBoundingRect().contains(pos):
        mousePoint = view2.mapSceneToView(pos)
        index = int(mousePoint.x())
        vLine.setPos(mousePoint.x())
        hLine.setPos(mousePoint.y())
proxy = pg.SignalProxy(view2.scene().sigMouseMoved, rateLimit=60, slot=mouseMoved)


win.show()
update()


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
