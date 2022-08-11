#!/usr/bin/env python3

# MIT License
# 
# Copyright (c) 2020 H.Saido <saido.nv@gmail.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from smbus2 import SMBus


STATUS_REG = 0x00
MEAS_MODE_REG = 0x01
ALG_RESULT_DATA_REG = 0x02
ENV_DATA_REG = 0x05
APP_START_REG = 0xF4

STATUS_DATA_READY_BIT = (1 << 3)

MODE_1SEC = 0x01
MODE_10SEC = 0x02
MODE_60SEC = 0x03
MODE_250MS = 0x04


class CCS811:
    """CCS811: ultra-low power digital gas sensor
    https://cdn.sparkfun.com/assets/learn_tutorials/1/4/3/CCS811_Datasheet-DS000459.pdf

    The equivalent CO2 (eCO2) output range for CCS811 is from
    400ppm to 8192ppm.
    The Total Volatile Organic Compound (TVOC) output range for
    CCS811 is from 0ppb to 1187ppb. 

    MODE settings: 1 sec interval, Disable interrupt

    param:
        bus_num (int): i2c bus number
        i2c_address (int): Address of CCS811
    """
    def __init__(self, bus_num=1, i2c_address=0x5b):
        self.i2c = SMBus(bus_num)
        self.i2c_address = i2c_address

        self.status = 0
        self.TVOC = 0
        self.eCO2 = 0

        # Write empty to APP_START to boot.
        self.i2c.write_i2c_block_data(self.i2c_address, APP_START_REG, [])
        # settings: 1 sec interval, Disable interrupt
        meas_mode = (MODE_1SEC << 4) | (0 << 3)
        self.i2c.write_byte_data(self.i2c_address, MEAS_MODE_REG, meas_mode)


    @property
    def ready(self):
        """Return data ready status
        """
        status = self.i2c.read_byte_data(self.i2c_address, STATUS_REG)
        if (status & STATUS_DATA_READY_BIT) == 0:
            return False
        else:
            return True


    def compensate(self, humidity=50.0, temperature=25.0):
        """Set environment value for compensate
        param:
            humidity (float): %
            temperature (float): Celsius degree

        The internal algorithm uses these values (or default values if
        not set by the application) to compensate for changes in
        relative humidity and ambient temperature.
        """
        h = int(humidity*512)
        t = int((temperature+25)*512)
        env_data = [(h>>8), (h&0xFF), (t>>8), (t&0xFF)]
        self.i2c.write_i2c_block_data(self.i2c_address, ENV_DATA_REG, env_data)


    def update(self):
        """Update TVOC and eCO2 values
        The values will update if data are available(ready).
        """
        if (self.ready):
            data = self.i2c.read_i2c_block_data(self.i2c_address, ALG_RESULT_DATA_REG, 8)
            co2 = (data[0]<<8) | (data[1])
            voc = (data[2]<<8) | (data[3])
            # Check range of the values
            # Skip update the values, when the values sometimes out of range.
            if 400 < co2 < 8192:
                self.eCO2 = co2
            if 0 < voc < 1187:
                self.TVOC = voc


    def get(self):
        """Return TVOC and eCO2 values
        """
        self.update()
        return self.TVOC, self.eCO2


if __name__ == '__main__':
    from time import sleep
    from bme280 import BME280

    ccs811 = CCS811()

    bme280 = BME280()
    p, t, h = bme280.get()
    ccs811.compensate(h, t)

    while(True):
        try:
            voc, co2 = ccs811.get()
            print(f"TVOC:{voc:4d} ppb, eCO2:{co2:4d} ppm")
            sleep(1)
        except KeyboardInterrupt:
            break
