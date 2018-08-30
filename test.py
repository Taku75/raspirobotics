import pigpio
from omni_whiles import Omni_Control
from motor import Motor
import msvcrt
import time
from decimal import Decimal

"""pi = pigpio.pi()
ur_motor = Motor(pi,14,15)
ul_motor = Motor(pi,17,18)
dr_motor = Motor(pi,22,23)
dl_motor = Motor(pi,9,25)

motors = [ur_motor, ul_motor, dl_motor, dr_motor]
omni_control = Omni_Control(motors, 0.5)"""
vx = 0
vy = 0
omega = 0
while True:
    time.sleep(0.5)
    if msvcrt.kbhit():
        key = msvcrt.getch()
        print(key)
        if key == b'\00':
            continue
        key = key.decode()
        print("pushed {0}".format(key))

        if key == 'd':
            vx += 0.10
            if vx > 1.0:
                vx = 1.0
        elif key == 'a':
            vx -= 0.10
            if vx < -1.0:
                vx = -1.0

        if key == 'w':
            vy += 0.10
            if vy > 1.0:
                vy = 1.0
        elif key == 's':
            vy -= 0.10
            if vy < -1.0:
                vy = -1.0

        if key == 'q':
            omega = 1.0
        elif key == 'e':
            omega = -1.0
    else:
        if vx > 0.01:
            vx -= 0.10
        elif vx < -0.01:
            vx += 0.10
        else:
            vx = 0

        if vy > 0.01:
            vy -= 0.10
        elif vy < -0.01:
            vy += 0.10
        else:
            vy = 0

        omega = 0.0

    print(str(vx) + " " + str(vy) + " " + str(omega))

        #omni_control.setspeed(vx, vy, omega)
