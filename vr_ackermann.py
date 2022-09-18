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
        self.speed_controller = PIDSat(8.0, 4.0, 0, 5, True)                                # kp = 8, ki = 4, ki = 0, saturation = 5 + anti-windup
        self.polar_controller = Polar2DController(5.0, 1.5, 40.0, math.pi/3)                # linear kp = 5, angular kp = 40, max linear speed = 1.5 m/s, max angular speed = 60 gd/s      

        self.path_controller = Path2D(1.5, 2, 2, 0.02)                                      # max speed = 1.5 m/s, acc = dec = 2 m/s^2, treshold = 2 cm
        self.path_controller.set_path([(0.0, 0.0)])
        
        (x, y, _) = self.get_pose()
        self.path_controller.start((x,y))
        self.last_target = (0,0)
        self.target_reached = False

        self.received_path = []
        self.draw_path = False
        self.plot_vars = False

        self.world = World()

        self.generated_blocks = False
        self.held_block = None

        self.phidias_agent = ''
        start_message_server_http(self)

        # the current command will be sent to the GUI and shown to the user
        self.current_cmd = 'Waiting for command from AI server...'
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
            self.register_plot(target, vref, steering)
        else:
            if not(self.target_reached):
                self.target_reached = True
                if self.phidias_agent != '':
                    print("[ACKERMANN] : Target reached")
                    # sends a signal to AI server when the robot reaches target/intermediate node
                    Messaging.send_belief(self.phidias_agent, 'target_got', [], 'robot')
                    if self.plot_vars is True: self.show_plot()

        return True

    def get_pose(self):
        return self.car.get_pose()

    def get_speed(self):
        return (self.car.v, self.car.w)

    def on_belief(self, _from, name, terms):
        print(_from, name, terms)
        self.phidias_agent = _from

        # when AI server tells the client to reach a node
        if name == 'go_to_node':
            for n in self.world.nodes:
                if terms[0] == n[0]: 
                    self.last_target = pixel2meter(n[1],n[2],30,600,1130)
                    self.received_path.append(self.last_target)
            print('[CURRENT PATH] :', self.received_path)
            self.path_controller.set_path([(self.last_target[0], self.last_target[1])])
            (x, y, _) = self.get_pose()
            self.path_controller.start((x,y))
            self.target_reached = False

            if terms[0][0] == 'T':
                if terms[0][1] == 'r':  self.current_cmd = 'Going to Red Tower'
                elif terms[0][1] == 'g':  self.current_cmd = 'Going to Green Tower'
                elif terms[0][1] == 'b':  self.current_cmd = 'Going to Blue Tower'
            else:
                self.current_cmd = 'Going to node ' + terms[0]

        # when the user enables via AI server the fuction to draw the path in the GUI
        if name == 'draw_path':
            self.draw_path = not(self.draw_path)
            if self.draw_path is True: 
                print('[ACKERMANN] : Path will be drawned')
                self.current_cmd = 'Path drawing function enabled'
            else:
                print('[ACKERMANN] : Path will not be drawned')
                self.current_cmd = 'Path drawing function disabled'

        # when the user enables via AI server the fuction to plot the variables
        if name == 'plot_vars':
            self.plot_vars = not(self.plot_vars)
            if self.plot_vars is True: 
                print('[ACKERMANN] : Variables will be plotted')
                self.current_cmd = 'Variables plotting function enabled'
            else:
                print('[ACKERMANN] : Variables will not be plotted')
                self.current_cmd = 'Variables plotting function disabled'

        # when the user generates a certain number of blocks via AI server
        if name == 'generate_blocks':
            if len(terms) == 0:
                self.world.generateBlocks()
            else:
                self.world.generateBlocks(terms[0])
            self.current_cmd = 'The blocks have been detected'

        # when the AI server tells the client to calculate the distance (in meters)
        # between the robot and the nearest node
        if name == 'sense_distance':
            self.calculate_distance()

        # when the AI server tells the client to detect the color located in the
        # nearest block
        if name == 'sense_color':
            self.detect_color()

        # when the AI server tells the robot's client to take the block (if present)
        # in the current node
        if name == 'send_held_block':
            for block in self.world.blocks:
                if block.in_node == terms[0]:
                    self.take_block(block)

        # when the AI server tells the robot's client to release the block
        # in the reached tower
        if name == 'releaseBlockToTower':
            self.release_block()

    # used to bring the last command to GUI, in order to paint it
    def get_current_cmd(self):
        return self.current_cmd

    def register_plot(self, target, vref, steering):
        (x, y, _) = self.get_pose()
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

    def show_plot(self):
        self.plotter.plot(['t', 'time'], [['vref', 'VRef'], ['v', 'V']])
        self.plotter.plot(['t', 'time'], [['steering', 'Steering'], ['w', 'W']])
        self.plotter.plot(['t', 'time'], [['x_target', 'X Target'], ['x', 'X']])
        self.plotter.plot(['t', 'time'], [['y_target', 'Y Target'], ['y', 'Y']])
        self.plotter.show()
  
    def calculate_distance(self):
        print("[ACKERMANN] : Calculating distance from nearest block")
        dist = self.world.closestBlockDistance(self.get_pose())
        if dist is None: params = []
        else: params = [dist]
        Messaging.send_belief(self.phidias_agent, 'distance', params, 'robot')

    def detect_color(self):
        print("[ACKERMANN] : Detecting nearest block color")
        color = self.world.closestBlockColor(self.get_pose())
        if color is None: params = []
        else: params = [color]
        Messaging.send_belief(self.phidias_agent, 'color', params, 'robot')

    def take_block(self, block):
        self.held_block = block
        self.world.removeBlock(block)

    def release_block(self):
        if self.held_block.getColor() == 'red': self.world.red_tower.addBlock()
        elif self.held_block.getColor() == 'green': self.world.green_tower.addBlock()
        elif self.held_block.getColor() == 'blue': self.world.blue_tower.addBlock()
        self.held_block = None

    

if __name__ == '__main__':
    cart_robot = AckermannRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot, cart_robot.world, 'ackermann_robot_2d.png')
    sys.exit(app.exec_())