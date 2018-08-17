import pigpio
from omni_whiles import Omni_Control
from motor import Motor

pi = pigpio.pi()
ur_motor = Motor(pi,)
ul_motor = Motor(pi,)
dr_motor = Motor(pi,)
dl_motor = Motor(pi,)

motors = [ur_motor, ul_motor, dl_motor, dr_motor]

while True:
    Omni_Control(motors, )
    
