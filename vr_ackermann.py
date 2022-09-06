import sys
sys.path.insert(0, 'lib')

from models.cart2d import *
from models.robot import *
from models.virtual_robot import *
from controllers.standard import *
from controllers.control2d import *
from data.plot import *
from gui.gui_2d import *

from PyQt5.QtWidgets import QApplication

class AckermannRobot(RoboticSystem):
    def __init__(self):
        super().__init__(1e-3)
        # mass = 10 kg
        # friction = 0.8
        # radius = 2 cm
        # lateral distance = 15 cm
        self.car = AckermannSteering(10, 0.8, 0.02, 0.15)
        self.speed_controller = PIDSat(2.0, 2.0, 0, 5, True)
        self.polar_controller = Polar2DController(2.0, 1.5, 10.0, math.pi/4)
        
        self.trajectory = StraightLine2DMotion(1.5, 2, 2)       # da sostituire con path2D
        (x,y,_) = self.get_pose()
        self.trajectory.start_motion((x, y), (0.1, 0.3))

        self.plotter = DataPlotter()
    
    def run(self):
        (x_target, y_target) = self.trajectory.evaluate(self.delta_t)
        (vref, steering) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())

        (v, w) = self.get_speed()
        (x,y,_) = self.get_pose()

        torque = self.speed_controller.evaluate(self.delta_t, vref, v)

        self.car.evaluate(self.delta_t, torque, steering)

        self.plotter.add('t', self.t)
        self.plotter.add('x', x)
        self.plotter.add('y', y)
        self.plotter.add('x_target', x_target)
        self.plotter.add('y_target', y_target)
        self.plotter.add('v', v)
        self.plotter.add('w', w)
        self.plotter.add('vref', vref)
        self.plotter.add('steering', steering)

        if self.t > 5:
            self.plotter.plot(['t', 'time'], [['vref', 'VRef'], ['v', 'V']])
            self.plotter.plot(['t', 'time'], [['steering', 'Steering'], ['w', 'W']])
            self.plotter.plot(['t', 'time'], [['x_target', 'X Target'], ['x', 'X']])
            self.plotter.plot(['t', 'time'], [['y_target', 'Y Target'], ['y', 'Y']])
            self.plotter.show()
            return False

        return True

    def get_pose(self):
        return self.car.get_pose()

    def get_speed(self):
        return (self.car.v, self.car.w)

if __name__ == '__main__':
    cart_robot = AckermannRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot, 'ackermann_robot_2d.png')
    sys.exit(app.exec_())