from time import sleep
import pigpio

class BMX055:
    def __init__(self, pi, bus, acc_address, gyro_address, mag_address):
        self.__bus = bus
        self.pi = pigpio.pi()
        self.Acc_handle = self.pi.i2c_open(self.__bus, acc_address)
        self.Gyro_handle = self.pi.i2c_open(self.__bus, gyro_address)
        self.Mag_handle = self.pi.i2c_open(self.__bus, mag_address)
        self.acc = Acc(pi, self.Acc_handle)
        self.gyro = Gyro(pi, self.Gyro_handle)
        self.mag = Mag(pi, self.Mag_handle)

# from stackoverflow J.F. Sebastian
def _twos_comp(val, bits=8):
    """
    compute the 2's complement of int val with bits
    """
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

class Acc:
    """
    Class for BMX055 accelerometer
    """

    def __init__(self, pi, handle):
        """
        Initializes with an I2C object and address as arguments.
        """
        self.pi = pi
        self._handle = handle
        self.__LPW = 0x00
        self.range = 2
        self.filter_bw = 8
        self.compensation()

    def read_accel(self, register: int, shift: int) -> float:
        """
        Returns acceleromter data from address.
        """
        LSB = self.pi.i2c_read_byte_data(self._handle, register)
        MSB = self.pi.i2c_read_byte_data(self._handle, register+1)
        return (-((((MSB << 8) + (LSB & (0xFF << shift))) >> shift) & (1 << (7+shift))) | ((((MSB << 8) + (LSB & (0xFF << shift))) >> shift) & ~(1 << (7+shift)))) * self._resolution

    def temperature(self) -> float:
        """
        Returns temperature in degrees C.
        """
        return self.pi.i2i2c_write_byte_data(self._handle, 0x08)/2 + 23

    @property
    def range(self) -> int:
        """
        Returns the accelerometer range
        """
        return {3:2, 5:4, 8:8, 12:16}[self.pi.i2c_read_byte_data(self._handle, 0x0F)]


    @range.setter
    def range(self, accel_range: int):
        """
        Sets the accelermoter range to 2, 4, 8 or 16g
        """

        range_mode = {2:0x03, 4:0x05, 8:0x08, 16:0x0C}
        try:
            range_byte = range_mode[accel_range]
        except KeyError:
            raise ValueError('invalid range, use 2, 4, 8 or 16')
        self.pi.i2c_write_byte_data(self._handle, 0x0F, range_byte)
        self._resolution = {2:0.98, 4:1.95, 8:3.91, 16:7.81}[accel_range]


    @property
    def filter_bw(self) -> int:
        """
        Returns the filter bandwith.
        """
        return 2**(self.pi.i2c_read_byte_data(self._handle, 0x10)-5)

    @filter_bw.setter
    def filter_bw(self, freq: int):
        """
        Sets the filter bandwith to 8, 16, 32, 64, 128, 256, 512 or 1024Hz.
        """
        freq_mode = {8:0x08, 16:0x09, 32:0x0A, 64:0x0B, 128:0x0C, 256:0x0D, 512:0x0E, 1024:0x0F}
        try:
            freq_byte = freq_mode[freq]
        except:
            raise ValueError('invalid filter bandwith, use 8, 16, 32, 64, 128, 256, 512 or 1024')
        self.pi.i2c_write_byte_data(self._handle, 0x10, freq_byte)

    def compensation(self, active=None) -> bool:
        """
        With no arguments passed, runs fast compensation.
        With boolean argument passe, activates or deactivates slow compensation.
        """
        accel_range = self.range
        self.range = 2
        self.pi.i2c_write_byte_data(self._handle, 0x37, 0x21) # settings x0y0z1 10Hz
        self.pi.i2c_write_byte_data(self._handle, 0x36, 0x80) # reset
        if active is None:  # trigger fast comp
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x00) # deactivate slow comp
            active = False
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x20) # x
            sleep(0.1)
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x40) # y
            sleep(0.1)
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x60) # z
            sleep(0.1)
        elif active:        # activate slow comp
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x07)
        elif not active:    # deactivate slow comp
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x00)
        else:
            raise TypeError('pass a boolean or no argument')
        self.range = accel_range
        return active

    @property
    def x(self) -> float:
        """
        Returns x acceleration in g.
        """
        return self.read_accel(0x02, 4)

    @property
    def y(self) -> float:
        """
        Returns y acceleration in g.
        """
        return self.read_accel(0x04, 4)

    @property
    def z(self) -> float:
        """
        Returns z acceleration in g.
        """
        return self.read_accel(0x06, 4)

    @property
    def xyz(self) -> tuple:
        """
        Returns x, y and z acceleration in g as tuple.
        """
        return (self.x, self.y, self.z)


class Gyro:
    """
    Class for gyroscape
    """

    def __init__(self, pi, handle):
        """
        Initializes with an I2C object and address as arguments.
        """
        self.buf = bytearray(64)
        self.pi = pi
        self._handle = handle
        self.range = 125
        self.filter_bw = 116
        self.compensation()

    def read_gyro(self, register: int, shift: int) -> float:
        """
        Returns gyro data from address.
        """
        LSB = self.pi.i2c_read_byte_data(self._handle, register)
        MSB = self.pi.i2c_read_byte_data(self._handle, register+1)
        return -((((MSB << 8) + (LSB & (0xFF << shift))) >> shift) & (1 << (7+shift))) | ((((MSB << 8) + (LSB & (0xFF << shift))) >> shift) & ~(1 << (7+shift)))

    @property
    def range(self) -> int:
        """
        Returns the gyro range
        """
        return int(2000/2**self.pi.i2c_read_byte_data(self._handle, 0x0F))

    @range.setter
    def range(self, gyro_range: int):
        """
        Sets the gyro range to 125, 250, 500, 1000 or 2000deg/s
        """
        range_mode = {125:0x04, 250:0x03, 500:0x02, 1000:0x01, 2000:0x000}
        try:
            range_byte = range_mode[gyro_range]
        except KeyError:
            raise ValueError('invalid range, use 125, 250, 500, 1000 or 2000')
        self.pi.i2c_write_byte_data(self._handle, 0x0F, range_byte)
        self._resolution = (2*gyro_range)/2**16


    @property
    def filter_bw(self) -> int:
        """
        Returns the filter bandwith.
        """
        return {0:523,1:230,2:116,3:47,4:23,5:12,6:64,7:32}[self.i2c.readfrom_mem(self.gyro_addr, 0x10, 1)[0]]

    @filter_bw.setter
    def filter_bw(self, freq: int):
        """
        Sets the filter bandwith to 12, 23, 32, 47, 64, 116, 230 or 523Hz.
        """
        freq_mode = {12:0x05, 23:0x04, 32:0x07, 47: 0x03, 64:0x06, 116:0x02, 230:0x01, 523:0x00}
        try:
            freq_byte = freq_mode[freq]
        except:
            raise ValueError('invalid filter bandwith, use 12, 23, 32, 47, 64, 116, 230 or 523')
        self.pi.i2c_write_byte_data(self._handle, 0x10, freq_byte)

    def compensation(self, active=None) -> bool:
        """
        With no arguments passed, runs fast compensation.
        With boolean argument passe, activates or deactivates slow compensation.
        """
        gyro_range = self.range
        self.range = 125
        self.pi.i2c_write_byte_data(self._handle, 0x37, 0x21) # settings x0y0z1 10Hz
        self.pi.i2c_write_byte_data(self._handle, 0x36, 0x80) # reset
        if active is None:  # trigger fast comp
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x00) # deactivate slow comp
            active = False
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x20) # x
            sleep(0.1)
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x40) # y
            sleep(0.1)
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x60) # z
            sleep(0.1)
        elif active:        # activate slow comp
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x07)
        elif not active:    # deactivate slow comp
            self.pi.i2c_write_byte_data(self._handle, 0x36, 0x00)
        else:
            raise TypeError('pass a boolean or no argument')
        self.range = gyro_range
        return active

    @property
    def x(self) -> float:
        """
        Returns x gyro in g.
        """
        return self.read_gyro(0x02, 0)

    @property
    def y(self) -> float:
        """
        Returns y gyro in g.
        """
        return self.read_gyro(0x04, 0)

    @property
    def z(self) -> float:
        """
        Returns z gyro in g.
        """
        return self.read_gyro(0x06, 0)

    @property
    def xyz(self) -> tuple:
        """
        Returns x, y and z gyro in g as tuple.
        """
        return (self.x, self.y, self.z)


class Mag:
    """
    Class for BMX055 magnetometer
    """

    def __init__(self, pi, handle):
        """
        Initalizes with an I2C object and adress as arguments.
        """
        self.pi = pi
        self._handle = handle
        self.pi.i2c_write_byte_data(self._handle, 0x4B, 0x01)
        #self.pi.i2c_write_byte_data(self._handle, 0x4C, 0x00)

    def read_mag(self, register: int, shift: int):
        LSB = self.pi.i2c_read_byte_data(self._handle, register)
        MSB = self.pi.i2c_read_byte_data(self._handle, register+1)
        return -((((MSB << 8) + (LSB & (0xFF << shift))) >> shift) & (1 << (7+shift))) | ((((MSB << 8) + (LSB & (0xFF << shift))) >> shift) & ~(1 << (7+shift)))

    @property
    def xMag(self) -> float:
        return (self.read_mag(0x42, 3))

    @property
    def yMag(self) -> float:
        return (self.read_mag(0x44, 3))

    @property
    def zMag(self) -> float:
        return (self.read_mag(0x46, 1))

    @property
    def xyzMag(self) -> tuple:
        return (self.xMag, self.yMag, self.zMag)
