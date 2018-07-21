import time

class PID:
    """
    PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()


    def clear(self):
        """Clear PID computations and confficients"""
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0


    def update(self, feedback_value):s
        """
        Calculate PID value for given reference feedback

        ..math::
            u(t) = Kp*e(t) + Ki*int_{0}^{t} e(t)dt + Kd de(t)/dt

        ..figure:: image/pid.png
            :align  center
        """
        error = self.SetPoint - feedback_value
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if(delta_time >= self.sample_time):
            self.PTerm = self.Kp *error
            self.ITerm += error * delta_time

            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.Dterm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.Dterm)

    def setKp(self, p_gain):
        self.Kp = p_gain

    def setKi(self, i_gain):
        self.Ki = i_gain

    def setKd(self, d_gain):
        self.Kd = d_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def SampleTime(self, sample_time):
        self.sample_time = sample_time
