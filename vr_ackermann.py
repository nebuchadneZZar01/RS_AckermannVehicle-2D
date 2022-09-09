import sys
sys.path.insert(0, 'lib')

from models.cart2d import *
from models.robot import *
from models.virtual_robot import *
from controllers.standard import *
from controllers.control2d import *
from data.plot import *
from gui.gui_2d import *

from utils import *
from block import *

from random import seed
from random import randint

from phidias.phidias_interface import *

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

        self.path_controller = Path2D(1.5, 2, 2, 0.01)
        self.path_controller.set_path([(0.0, 0.0)])
        
        (x, y, _) = self.get_pose()
        self.path_controller.start((x,y))
        self.target_reached = False

        self.received_path = []

        self.graph = readNodesFile('nodes.txt')
        self.blocks = [ ]
        self.generated_blocks = False

        self.phidias_agent = ''
        start_message_server_http(self)

        self.plotter = DataPlotter()
    
    def run(self):
        pose = self.get_pose()
        target = self.path_controller.evaluate(self.delta_t, pose)
        
        if target is not None:
            (x_target, y_target) = target
            (vref, steering) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())

            (v, w) = self.get_speed()
            (x, y, _) = self.get_pose()

            torque = self.speed_controller.evaluate(self.delta_t, vref, v)

            self.car.evaluate(self.delta_t, torque, steering)
        else:
            if not(self.target_reached):
                self.target_reached = True
                if self.phidias_agent != '':
                    print("Target")
                    Messaging.send_belief(self.phidias_agent, 'targetReached', [], 'robot')

        return True

    def get_pose(self):
        return self.car.get_pose()

    def get_speed(self):
        return (self.car.v, self.car.w)

    def on_belief(self, _from, name, terms):
        print(_from, name, terms)
        self.phidias_agent = _from
        if name == 'go_to_node':
            print(terms)
            for n in self.graph:
                if terms[0] == n[0]: 
                    next_target = pixel2meter(n[1],n[2])
                    self.received_path.append(next_target)
            print(self.received_path)
            self.path_controller.set_path([(next_target[0], next_target[1])])
            (x, y, _) = self.get_pose()
            self.path_controller.start((x,y))
            self.target_reached = False

        if name == 'generate_blocks':
            self.generate_blocks()

    def get_plot(self, target, vref, steering):
        (x, y) = self.get_pose()
        (x_target, y_target) = target
        (v, w) = self.get_speed()
        (vref, steering)

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

    def generate_blocks(self):
        self.blocks.clear()

        for n in self.graph:
            if n[3] == True: 
                c = randint(0,2)
                in_n = n[0]
                (x_M, y_M) = pixel2meter(n[1],n[2])
                block = Block(in_n,n[1],n[2],x_M,y_M,c)
                self.blocks.append(block)

                if self.generated_blocks == False:
                    self.generated_blocks = True

        print("There are " + str(len(self.blocks)) + " blocks")


if __name__ == '__main__':
    #seed(1)
    cart_robot = AckermannRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot, 'ackermann_robot_2d.png')
    sys.exit(app.exec_())