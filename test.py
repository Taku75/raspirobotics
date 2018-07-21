import pigpio
from motor import Motor

pi=pigpio.pi()
motor=Motor(pi, 12, 13)
