import sys
import math
import pathlib

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QWidget
import utils

BCOL = {
    'red': QtGui.QColor(255,0,0),
    'green': QtGui.QColor(0,255,0),
    'blue': QtGui.QColor(0,0,255),
    'gray': QtGui.QColor(148,148,148),
    'brown': QtGui.QColor(165,42,42)
}

L = 10  # node size

class CartWindow(QWidget):

    def __init__(self, _compound_sys, _img = 'mobile_robot_2d.png'):
        super(CartWindow, self).__init__()
        self.compound_system = _compound_sys
        self.image = _img
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 1280, 720)
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
        qp.drawLine(50, 600, 1150, 600)
        qp.drawLine(50, 600, 50, 50)
        qp.drawLine(50, 50, 1150, 50)
        qp.drawLine(1150, 50, 1150, 600)

        qp.drawText(1170, 20, "X  = %6.3f m" % (x))
        qp.drawText(1170, 40, "Y  = %6.3f m" % (y))
        qp.drawText(1170, 60, "Th = %6.3f deg" % (math.degrees(theta)))
        qp.drawText(1170, 80, "T  = %6.3f s" % (self.compound_system.t))

        drawGraph(qp)

        s = self.robot_pic.size()

        x_pos = int(50 + x*1150 - s.width() / 2)
        y_pos = int(600 - y*1150 - s.height() / 2)

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
        qp.drawEllipse(node[1], node[2], L, L)


def main():
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
