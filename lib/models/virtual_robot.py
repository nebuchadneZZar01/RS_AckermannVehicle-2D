import math
from data.geometry import *

class VirtualRobot:
    ACCEL = 0
    CRUISE = 1
    DECEL = 2
    TARGET = 3
    def __init__(self, _p_target, _vmax, _acc, _dec):
        self.p_target = _p_target
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec
        self.v = 0 # current speed
        self.p = 0 # current position
        self.phase = VirtualRobot.ACCEL
        self.decel_distance = 0.5 * _vmax * _vmax / _dec

    def evaluate(self, delta_t):
        if self.phase == VirtualRobot.ACCEL:
            self.p = self.p + self.v * delta_t \
                     + self.accel * delta_t * delta_t / 2
            self.v = self.v + self.accel * delta_t
            distance = self.p_target - self.p
            if distance < 0:
                distance = 0
            if self.v >= self.vmax:
                self.v = self.vmax
                self.phase = VirtualRobot.CRUISE
            elif distance <= self.decel_distance:
                v_exp = math.sqrt(2 * self.decel * distance)
                if v_exp < self.v:
                    self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.CRUISE:
            self.p = self.p + self.vmax * delta_t
            distance = self.p_target - self.p
            if distance <= self.decel_distance:
                self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.DECEL:
            self.p = self.p + self.v * delta_t \
                     - self.decel * delta_t * delta_t / 2
            self.v = self.v - self.decel * delta_t
            if self.p >= self.p_target:
                self.v = 0
                self.p = self.p_target
                self.phase = VirtualRobot.TARGET


# ------------------------------------------------------------

class VirtualRobot2D:
    ACCEL = 0
    CRUISE = 1
    DECEL = 2
    TARGET = 3
    def __init__(self, _vmax, _acc, _dec):
        self.linear_trajectory = VirtualRobot(0, _vmax, _acc, _dec)

    def set_target(self, starting, target):
        self.starting = starting
        dx = target[0] - starting[0]
        dy = target[1] - starting[1]

        self.linear_target = math.hypot(dx,dy)
        self.target_heading = math.atan2(dy,dx)

        self.linear_trajectory.p_target = self.linear_target


    def evaluate(self, delta_t):
        self.linear_trajectory.evaluate(delta_t)
        x = self.starting[0] + self.linear_trajectory.p * math.cos(self.target_heading)
        y = self.starting[1] + self.linear_trajectory.p * math.sin(self.target_heading)
        return (x,y)