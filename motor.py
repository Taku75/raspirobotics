from pwm import Pwm
import pigpio

class Motor(Pwm):
    def __init__(self, pi, pwm_pin, io_pin, frequency = 2000, range = 1000):
        super().__init__(pi, pwm_pin, frequency, range)
        self._io_pin = io_pin
        self._direction = 0
        self._speed = 0
        self.pi.set_mode(self._io_pin, pigpio.OUTPUT)
        self.pi.write(self._io_pin, self._direction)

    @property
    def direction(self):
        if self._direction == 0:
            return 'cw'
        else:
            return 'ccw'

    @direction.setter
    def direction(self, direction):
        self._direction = direction
        self.pi.write(self._io_pin, self._direction)

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        if(speed > super().range):
            speed = self.range
        elif(speed < -1*self.range):
            speed = -1*self.range
        else:
            pass

        self.direction = (speed < 0)
        self.duty = abs(speed)
