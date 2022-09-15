import sys
import math
import pathlib

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QWidget
from utils import *

W = 1280    # window
H = 720     # size

R = 10      # node radius
L = 15      # block edge

BCOL = {
    'red': QtGui.QColor(255,0,0),
    'green': QtGui.QColor(0,255,0),
    'blue': QtGui.QColor(0,0,255),
    'gray': QtGui.QColor(148,148,148),
    'brown': QtGui.QColor(165,42,42),
    'black': QtGui.QColor(0,0,0),
    'white': QtGui.QColor(255,255,255),
    'transparent': QtGui.QColor(0,0,0,0)
}

colors = ['red', 'green', 'blue']

class CartWindow(QWidget):
    def __init__(self, _compound_sys, _world, _img = 'mobile_robot_2d.png'):
        super(CartWindow, self).__init__()
        self.compound_system = _compound_sys
        self.world = _world
        self.image = _img
        self.initUI()
        self.setMouseTracking(True)

        self.path = QtGui.QPainterPath()

    def initUI(self):
        self.setGeometry(0, 0, W, H)
        self.setWindowTitle('Ackermann 2D Simulation')
        self.show()

        current_path = pathlib.Path(__file__).parent.resolve()
        image = str(current_path) + '/../icons/' + self.image

        self.img = image
        self.red_image = str(current_path) + '/../icons/' + 'ackermann_robot_redB.png'
        self.green_image = str(current_path) + '/../icons/' + 'ackermann_robot_greenB.png'
        self.blue_image = str(current_path) + '/../icons/' + 'ackermann_robot_blueB.png'

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
        qp.setPen(BCOL['white'])
        qp.setBrush(BCOL['white'])
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
        qp.drawText(1150, 160, "Red = " + str(self.world.getBlocksNumberByColor(0)))
        qp.drawText(1150, 180, "Green = " + str(self.world.getBlocksNumberByColor(1)))
        qp.drawText(1150, 200, "Blue = " + str(self.world.getBlocksNumberByColor(2)))
        qp.drawText(1150, 240, "Blocks in Towers:")
        qp.drawText(1150, 260, "Red = " + str(self.world.red_tower.n_blocks) + "/3")
        qp.drawText(1150, 280, "Green = " + str(self.world.green_tower.n_blocks) + "/3")
        qp.drawText(1150, 300, "Blue = " + str(self.world.blue_tower.n_blocks) + "/3")
        qp.drawText(1150, 340, "Legend:")
        qp.drawText(1160, 360, " = Graph Node")
        qp.drawText(1160, 380, " = Block")
        qp.drawText(1160, 400, " = Tower")
        qp.drawText(1160, 420, " = Obstacle")
        qp.drawText(1180, 449, " = Start point")
        qp.drawText(1150, 490, "Functionalities:")
        if self.compound_system.plot_vars: qp.drawText(1150, 510, "Plot Variables: [ON]")
        else: qp.drawText(1150, 510, "Plot Variables: [OFF]")
        if self.compound_system.draw_path: qp.drawText(1150, 530, "Draw Path: [ON]")
        else: qp.drawText(1150, 530, "Draw Path: [OFF]")

        self.paintCommand(qp, self.compound_system.get_current_cmd())

        self.drawLegend(qp)

        self.drawGraph(qp, self.world)

        self.drawStartArrow(qp)

        s = self.robot_pic.size()

        if self.world.generated_blocks == True:
            for block in self.world.blocks:
                self.drawBlock(qp, block.x_P_pos, block.y_P_pos, block.color)

        for obstacle in self.world.obstacles:
            self.drawObstacle(qp, obstacle[0], obstacle[1])

        if self.compound_system.held_block:
            if self.compound_system.held_block.getColor() == 'red': self.robot_pic = QtGui.QPixmap(self.red_image)
            elif self.compound_system.held_block.getColor() == 'green': self.robot_pic = QtGui.QPixmap(self.green_image)
            elif self.compound_system.held_block.getColor() == 'blue': self.robot_pic = QtGui.QPixmap(self.blue_image)
        else: self.robot_pic = QtGui.QPixmap(self.img)

        # draws block in a tower if present
        if self.world.red_tower is not None:
            if self.world.red_tower.n_blocks >= 1: 
                self.drawBlock(qp, self.world.red_tower.x_P_pos, self.world.red_tower.y_P_pos, 0)

        if self.world.green_tower is not None:
            if self.world.green_tower.n_blocks >= 1: 
                self.drawBlock(qp, self.world.green_tower.x_P_pos, self.world.green_tower.y_P_pos, 1)

        if self.world.blue_tower is not None:
            if self.world.blue_tower.n_blocks >= 1:
                self.drawBlock(qp, self.world.blue_tower.x_P_pos, self.world.blue_tower.y_P_pos, 2)

        x_pos = int(30 + x*1130 - s.width() / 2)
        y_pos = int(600 - y*1130 - s.height() / 2)

        # draws path if enabled from AI (for debugging)
        if self.compound_system.draw_path: 
            self.drawPoints(qp, x_pos, y_pos)

        t = QtGui.QTransform()
        t.translate(x_pos + s.width()/2, y_pos + s.height()/2)
        t.rotate(-math.degrees(theta))
        t.translate(-(x_pos + s.width()/2), - (y_pos + s.height()/2))

        qp.setTransform(t)
        qp.drawPixmap(x_pos,y_pos,self.robot_pic)

        qp.end()

    # for debugging
    def mousePressEvent(self, a0: QtGui.QMouseEvent):
        print('Mouse pressed:', a0.x(), a0.y())

    def drawPoints(self, qp, x, y):
        qp.setPen(BCOL['red'])
        qp.setBrush(BCOL['transparent'])
        self.path.lineTo(x, y)
        qp.drawPath(self.path)

    def drawLegend(self, qp):
        # NODE
        qp.setPen(QtCore.Qt.NoPen)
        qp.setBrush(BCOL['gray'])
        qp.drawEllipse(1150, 350, R, R)

        # BLOCK
        qp.setPen(BCOL['black'])
        qp.setBrush(BCOL['red'])
        qp.drawRect(1150, 370, 10, 10)

        # TOWER
        qp.setPen(BCOL['red'])
        qp.setBrush(BCOL['transparent'])
        qp.drawRect(1150, 390, 10, 10)

        # OBSTACLE
        qp.setPen(BCOL['black'])
        qp.setBrush(BCOL['black'])
        qp.drawRect(1150, 410, 10, 10)

        # START
        qp.setPen(QtCore.Qt.NoPen)
        qp.setBrush(BCOL['brown'])
        qp.drawRect(1150, 440, 10, 10)
        points = [QtCore.QPoint(1160,430), QtCore.QPoint(1160,460), QtCore.QPoint(1180,445)]
        triangle = QtGui.QPolygon(points)
        qp.drawPolygon(triangle)

    def drawTower(self, qp, x_pos, y_pos, color):
        qp.setPen(BCOL[colors[color]])
        qp.setBrush(BCOL['transparent'])
        qp.drawRect(x_pos-int(L/2), y_pos-int(L/2), 30, 30)
    
    def drawStartArrow(self, qp):
        qp.setPen(QtCore.Qt.NoPen)
        qp.setBrush(BCOL['brown'])
        qp.drawRect(10,590,60,20)
        points = [QtCore.QPoint(70,570), QtCore.QPoint(70,630), QtCore.QPoint(100,600)]
        triangle = QtGui.QPolygon(points)
        qp.drawPolygon(triangle)

    def drawGraph(self, qp, world):
        qp.setPen(QtCore.Qt.NoPen)

        nodes = world.nodes 

        for node in nodes:
            if node[0] == 'START': 
                qp.setBrush(BCOL['brown'])
                qp.drawEllipse(node[1], node[2], R, R)
            elif node[0] == 'Tr': self.drawTower(qp,node[1],node[2],0)
            elif node[0] == 'Tg': self.drawTower(qp,node[1],node[2],1)
            elif node[0] == 'Tb': self.drawTower(qp,node[1],node[2],2)
            else: 
                qp.setBrush(BCOL['gray'])
                qp.drawEllipse(node[1], node[2], R, R)
    
    def drawBlock(self, qp, x_pos, y_pos, color):
        qp.setPen(BCOL['black'])
        qp.setBrush(BCOL[colors[color]])
        qp.drawRect(x_pos, y_pos, L, L)

    def drawObstacle(self, qp, x_pos, y_pos):
        qp.setPen(BCOL['black'])
        qp.setBrush(BCOL['black'])
        qp.drawRect(x_pos, y_pos, 20, 20)

    def paintCommand(self, qp, command):
        qp.drawText(450, 665, "COMMAND: " + command)


def main():
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
