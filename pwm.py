import pigpio

class PWM:
    def __init__(self, pi: pigpio.pi(), pwm_pin):
        self.pi = pi
        self.__pwm_pin = pwm_pin
        self.__pwm = 0

    @property
    def pwm(self):
        return self.__pwm

print("Hello!")
