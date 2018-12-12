''' Controller Module
Available Controllers:
- Dualshock 4 (DS4)
Requires:
- Pygame module
- Connected bluetooth controller
TODO:
- Check that controller is connected.
- Implement controller disconnect procedure.
'''

import threading
import pygame
from time import sleep

class DS4(threading.Thread):
    joystick = None
    name = None
    axes_count = 0
    axes = {}
    button_count = 0
    buttons = {}
    connected = False

    CONNECT_FREQUENCY = 1/10
    POLL_FREQUENCY = 20

    ''' PS4 buttons indices '''
    SQUARE = 0
    CROSS = 1
    CIRCLE = 2
    TRIANGLE = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    SHARE = 8
    OPTIONS = 9
    LEFT_ANALOG_PRESS = 10
    RIGHT_ANALOG_PRESS = 11
    PS4_ON_BUTTON = 12
    TOUCHPAD = 13

    ''' PS4 Axes indices '''
    LEFT_X_AXIS = 0
    LEFT_Y_AXIS = 1
    RIGHT_X_AXIS = 2
    RIGHT_Y_AXIS = 5

    def __init__(self):
        threading.Thread.__init__(self)


    def run(self):
        while True:
            print('Trying to connect.')
            self.connect()
            while self.connected:
                self.get_axes()
                self.get_buttons()
                sleep(1/self.POLL_FREQUENCY)
            sleep(1/self.CONNECT_FREQUENCY)
        return True


    def connect(self):
        ''' Initiate pygame and pygame.joystick module used for the controller. '''
        try:
            pygame.quit()
            pygame.init()
            pygame.joystick.init()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.name = self.joystick.get_name()

            self.axes_count = self.joystick.get_numaxes()
            for i in range (0, self.axes_count):
                self.axes[i] = 0.0

            self.button_count = self.joystick.get_numbuttons()
            for i in range (0, self.button_count):
                self.buttons[i] = 0
            self.connected = True
            return True
        except:
            self.connected = False
            return False

    def get_buttons(self):
        ''' Return the value for all buttons from the controller '''
        for i in range(self.joystick.get_numbuttons()):
            self.buttons[i] = self.joystick.get_button(i)
        return self.buttons


    def get_axes(self):
        ''' Return the value of all axes from the controller. '''
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axes[event.axis] = round(event.value, 2)
        return self.axes


    def is_connected(self):
        return self.connected

    def check_connection(self):
        ''' Check controller connection. Not working currently.'''
        pass


    def disconnect(self):
        ''' Quit the pygame module. '''
pygame.quit()
