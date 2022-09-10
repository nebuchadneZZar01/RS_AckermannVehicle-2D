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
from world import *

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
        self.last_target = (0,0)
        self.target_reached = False

        self.received_path = []

        self.world = World()

        self.blocks = [ ]
        self.generated_blocks = False

        self.red_held = False
        self.green_held = False
        self.blue_held = False

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
                    Messaging.send_belief(self.phidias_agent, 'target_got', [], 'robot')

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
            for n in self.world.nodes:
                if terms[0] == n[0]: 
                    self.last_target = pixel2meter(n[1],n[2])
                    self.received_path.append(self.last_target)
            print(self.received_path)
            self.path_controller.set_path([(self.last_target[0], self.last_target[1])])
            (x, y, _) = self.get_pose()
            self.path_controller.start((x,y))
            self.target_reached = False

        if name == 'generate_blocks':
            self.world.generateBlocks()

        if name == 'sense_distance':
            self.calculate_distance()

        if name == 'sense_color':
            self.detect_color()

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

    def calculate_distance(self):
        print("Calculating distance from nearest block")
        dist = self.world.closestBlockDistance(self.get_pose())
        Messaging.send_belief(self.phidias_agent, 'distance', [dist], 'robot')

    def detect_color(self):
        print("Detecting nearest block color")
        color = self.world.closestBlockColor(self.get_pose())
        Messaging.send_belief(self.phidias_agent, 'color', [color], 'robot')

if __name__ == '__main__':
    cart_robot = AckermannRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot, cart_robot.world, 'ackermann_robot_2d.png')
    sys.exit(app.exec_())