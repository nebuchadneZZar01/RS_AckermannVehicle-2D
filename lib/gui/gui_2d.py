import sys
import math
import pathlib

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QWidget
import utils

W = 1280    # window
H = 720     # size

R = 10      # node radius
L = 15      # block edge

BCOL = {
    'red': QtGui.QColor(255,0,0),
    'green': QtGui.QColor(0,255,0),
    'blue': QtGui.QColor(0,0,255),
    'gray': QtGui.QColor(148,148,148),
    'brown': QtGui.QColor(165,42,42)
}

colors = ['red', 'green', 'blue']

class CartWindow(QWidget):

    def __init__(self, _compound_sys, _img = 'mobile_robot_2d.png'):
        super(CartWindow, self).__init__()
        self.compound_system = _compound_sys
        self.image = _img
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, W, H)
        self.setWindowTitle('Ackermann 2D Simulation')
        self.show()

        current_path = pathlib.Path(__file__).parent.resolve()
        image = str(current_path) + '/../icons/' + self.image

        self.robot_pic = QtGui.QPixmap(image)

        self.delta_t = 1e-4 # 0.1ms of time-tick

        self._timer_painter = QtCore.QTimer(self)
        self._timer_painter.start(int(self.delta_t * 1000))
        self._timer_painter.timeout.connect(self.go)


    def go(self):
        if not(self.compound_system.step()):
            self._timer_painter.stop()
        self.update() # repaint window


    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setPen(QtGui.QColor(255,255,255))
        qp.setBrush(QtGui.QColor(255,255,255))
        qp.drawRect(event.rect())

        (x, y, theta) = self.compound_system.get_pose()

        qp.setPen(QtCore.Qt.black)
        qp.drawLine(30, 600, 1130, 600)
        qp.drawLine(30, 600, 30, 30)
        qp.drawLine(30, 30, 1130, 30)
        qp.drawLine(1130, 30, 1130, 600)

        qp.drawText(1150, 40, "X  = %6.3f m" % (x))
        qp.drawText(1150, 60, "Y  = %6.3f m" % (y))
        qp.drawText(1150, 80, "Th = %6.3f deg" % (math.degrees(theta)))
        qp.drawText(1150, 100, "T  = %6.3f s" % (self.compound_system.t))
        qp.drawText(1150, 140, "Remaining Blocks:")
        qp.drawText(1150, 160, "Red = ")
        qp.drawText(1150, 180, "Green = ")
        qp.drawText(1150, 200, "Blue = ")

        drawGraph(qp)

        s = self.robot_pic.size()

        if self.compound_system.generated_blocks == True:
            for block in self.compound_system.blocks:
                drawBlock(qp, block.x_P_pos, block.y_P_pos, block.color)

        x_pos = int(30 + x*1130 - s.width() / 2)
        y_pos = int(600 - y*1130 - s.height() / 2)

        t = QtGui.QTransform()
        t.translate(x_pos + s.width()/2, y_pos + s.height()/2)
        t.rotate(-math.degrees(theta))
        t.translate(-(x_pos + s.width()/2), - (y_pos + s.height()/2))

        qp.setTransform(t)
        qp.drawPixmap(x_pos,y_pos,self.robot_pic)

        qp.end()
    

def drawGraph(qp):
    qp.setPen(QtCore.Qt.NoPen)

    (nodes, links) = utils.readGraphFiles('nodes.txt', 'links.txt')

    for node in nodes:
        if node[0] == 'START': qp.setBrush(BCOL['brown'])
        elif node[0] == 'Tr': qp.setBrush(BCOL['red'])
        elif node[0] == 'Tg': qp.setBrush(BCOL['green'])
        elif node[0] == 'Tb': qp.setBrush(BCOL['blue'])
        else: qp.setBrush(BCOL['gray'])
        qp.drawEllipse(node[1], node[2], R, R)

def drawBlock(qp, x_pos, y_pos, color):
    qp.setPen(QtGui.QColor(0,0,0))
    qp.setBrush(BCOL[colors[color]])
    qp.drawRect(x_pos, y_pos, L, L)


def main():
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
