import pigpio
from motor import Motor
from typing import List
from math import *
import numpy as np

omuni=List[Motor]

class Omni_Control:

    ALPHA_ = pi()/4

    def __init__(self, motors: omuni, radius):
        self._motors = {"Right-Front":moters[0], "Right-Back":motors[1], "Left-Front":motors[2], "Left-Back":motors[3]}
        self._motors_key = list(_motors.keys())
        self._motor_list = list(_motors.values())
        self._speed = {"Right-Front":motors[0].speed, "Right-Back":motors[1].speed, "Left-Front":motors[2].speed, "Left-Back":motors[3].speed}
        self.__radius = radius
        self._range = motors[0].range


    @property
    def speed(self, key = "All": str):
        if key == "All":
            return self._speed
        return self._speed.get(key)

    @speed.setter
    def speed(self, vx, vy, omega):
        A = np.array([[-sin(self.ALPHA_), sin(self.ALPHA_), self.__radius],
                    [-cos(self.ALPHA_), -cos(self.ALPHA_), self.__radius],
                    [sin(self.ALPHA_), -sin(self.ALPHA_), self.__radius],
                    [cos(self.ALPHA_), cos(self.ALPHA_), self.__radius]])
        motor_array=np.array(self._motor_list)
        speed = np.dot(A, motor_array)
        self._motors[self._motors_key[0]].speed = speed[0]*self._range
        self._motors[self._motors_key[1]].speed = speed[1]*self._range
        self._motors[self._motors_key[2]].speed = speed[2]*self._range
        self._motors[self._motors_key[3]].speed = speed[3]*self._range
