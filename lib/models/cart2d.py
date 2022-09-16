import math
from data.geometry import *

class AckermannSteering:
    def __init__(self, _mass, _lin_friction, _r_traction, _lateral_wheelbase):
        self.M = _mass
        self.b = _lin_friction
        self.r_wheels = _r_traction
        self.l_wb = _lateral_wheelbase

        self.v = 0
        self.w = 0
        self.x = 0
        self.y = 0
        self.theta = 0


    def evaluate(self, delta_t, torque, steering_angle):
        _force = torque / self.r_wheels
        new_v = self.v * (1 - self.b * delta_t / self.M) + delta_t * _force / self.M

        if steering_angle == 0:
            new_w = 0
        else:
            curvature_radius = self.l_wb / math.tan(steering_angle)
            new_w = new_v / curvature_radius

        self.x = self.x + self.v * delta_t * math.cos(self.theta)
        self.y = self.y + self.v * delta_t * math.sin(self.theta)
        self.theta = self.theta + delta_t * self.w
        self.v = new_v
        self.w = new_w


    def get_pose(self):
        return (self.x, self.y, self.theta)


    def get_speed(self):
        return (self.v, self.w)