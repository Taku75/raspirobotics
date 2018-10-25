import pigpio

class Pwm:
    def __init__(self, pi, gpio_pin, frequency = 2000, range = 1000):
        self.__pwm_pin = gpio_pin
        self.__frequency = frequency
        self.__range = range
        self.__duty = 0
        self.pi = pi
        self.pi.set_PWM_frequency(self.__pwm_pin, self.__frequency)
        self.pi.set_PWM_range(self.__pwm_pin, self.__range)
        self.pi.set_PWM_dutycycle(self.__pwm_pin, self.__duty)

    @property
    def pwm_pin(self):
        return self.__pwm_pin

    @property
    def frequency(self):
        return self.__frequency

    @frequency.setter
    def frequency(self, frequency):
        self.__frequency = frequency
        self.pi.set_PWM_frequency(self.__pwm_pin, self.__frequency)

    @property
    def range(self):
        return self.__range

    @range.setter
    def range(self, range):
        self.__duty = int(self.__duty * range / self.__range)
        self.__range = range
        self.pi.set_PWM_range(self.__pwm_pin, self.__range)
        self.pi.set_PWM_dutycycle(self.__pwm_pin, self.__duty)

    @property
    def duty(self):
        return self.__duty

    @duty.setter
    def duty(self, duty):
        self.__duty = duty
        if(self.__duty > self.range):
            self.__duty = self.__range
        elif(self.__duty < 0):
            self.__duty = 0
        self.pi.set_PWM_dutycycle(self.__pwm_pin, self.__duty)
