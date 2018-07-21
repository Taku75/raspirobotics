import pigpio
from motor import Motor
from typing import List
import math
import numpy as np

omuni=List[Motor]

class Omuni_Control:

    def __init__(self, motors: omuni, ):
        self._motors = {"Right-Front":moters[0], "Right-Back":motors[1], "Left-Front":motors[2], "Left-Back":motors[3]}
        self._motors_key = list(_motors.keys())
        self._motor_list = list(_motors.values())
        self._speed = {"Right-Front":motors[0].speed, "Right-Back":motors[1].speed, "Left-Front":motors[2].speed, "Left-Back":motors[3].speed}


    @property
    def speed(self, key = "All": str):
        if key == "All":
            return self._speed
        return self._speed.get(key)

    @speed.setter
    def speed(self, vx, vy, omega):
        A = np.array([[-math.sin(), math.cos(), math.cos()],
                    [-math.cos(), -math.sin(), math.cos()],
                    [math.sin(), -math.cos(), math.cos()],
                    [math.cos(), math.sin(), math.cos()]])
        motor_array=np.array(self._motor_list)
        speed = np.dot(A, motor_array)
