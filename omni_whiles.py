import pigpio
from motor import Motor
from math import *
import numpy as np

class Omni_Control:
    def __init__(self, motors, radius):
        print(motors)
        self._motors = motors
        #self._motors_key = list(self._motors.keys())
        #self._motor_list = list(self._motors.values())
        self._speed = [motors[0].speed, motors[1].speed, motors[2].speed, motors[3].speed]
        self.__radius = radius
        self._range = motors[0].range
        self._vx = 0
        self._vy = 0
        self._omega = 0

    @property
    def speed(self, key = "All"):
        if key == "All":
            return self._speed


    def setspeed(self, vx, vy, omega):
        ALPHA_ = pi/4
        A = np.array([[-sin(ALPHA_), sin(ALPHA_), self.__radius],
                    [-cos(ALPHA_), -cos(ALPHA_), self.__radius],
                    [sin(ALPHA_), -sin(ALPHA_), self.__radius],
                    [cos(ALPHA_), cos(ALPHA_), self.__radius]])
        motor_array=np.array([vx,vy,omega])
        _speed = np.dot(A, motor_array)
        _speed = _speed.tolist()
        self._speed[0] = _speed[0]
        self._speed[1] = _speed[1]
        self._speed[2] = _speed[2]
        self._speed[3] = _speed[3]
        self._motors[0].speed = self._speed[0]*self._range
        self._motors[1].speed = self._speed[1]*self._range
        self._motors[2].speed = self._speed[2]*self._range
        self._motors[3].speed = self._speed[3]*self._range
