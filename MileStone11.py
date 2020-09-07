# -*- coding: utf-8 -*-
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import numpy as np

from pyqtgraph.dockarea import *

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
# Dock 1 - Preview image from camera & overlay #
w1 = pg.LayoutWidget()

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



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
