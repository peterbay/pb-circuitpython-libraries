# SPDX-FileCopyrightText: 2022 Petr Vavrin <pvavrin@gmail.com>
# SPDX-FileCopyrightText: Copyright (c) 2022 Petr Vavrin
#
# SPDX-License-Identifier: MIT
#

"""
# Usage:

from bh1750 import BH1750
sensor = BH1750(i2c)

# sensor.set(sensor.MODE, sensor.MODE_CONTINUOUSLY, sensor.RESOLUTION_LOW)
# sensor.set(sensor.MODE, sensor.MODE_CONTINUOUSLY, sensor.RESOLUTION_HIGH)
sensor.set(sensor.MODE, sensor.MODE_CONTINUOUSLY, sensor.RESOLUTION_HIGH_2)

while True:
    lux = sensor.get_value()
    if lux:
        print("%.4f Lux" % lux)

"""

from time import monotonic
import adafruit_bus_device.i2c_device as i2c_device

class BH1750:
    MODE = "mode",
    POWER = "power"
    RESOLUTION_LOW = "low"
    RESOLUTION_HIGH = "high"
    RESOLUTION_HIGH_2 = "high2"
    MODE_SINGLE = "single"
    MODE_CONTINUOUSLY = "continuously"

    def __init__(self, i2c_bus, address=0x23):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self.buffer = bytearray(2)
        self.measure_mode = self.MODE_CONTINUOUSLY
        self.resolution = self.RESOLUTION_HIGH_2
        self.set(self.POWER, True)
        self.time = 0
        self.time_wait = 0.130

    def write(self, opecode):
        self.buffer[0] = opecode
        with self.i2c_device as i2c:
            i2c.write(self.buffer, end=1)

    def get_config(self):
        return {
            "name": "BH1750",
            "type": "optical",
            "direction": "I",
            "description": "Ambient Light Sensor (1 - 65535 lx)",
            "controls": {
                "mode": {
                    "mode": "RW",
                    "values": [
                        {"type": "ENUM", "enums": [self.MODE_SINGLE, self.MODE_CONTINUOUSLY]},
                        {"type": "ENUM", "enums": [self.RESOLUTION_LOW, self.RESOLUTION_HIGH, self.RESOLUTION_HIGH_2]}
                    ],
                    "description": "Measurement mode and resolution"
                },
                "power": {
                    "mode": "RW",
                    "values": [
                        {"type": "INT", "min": 0, "max": 1}
                    ],
                    "description": "Power management"
                },
            },
            "value": {
                "mode": "R",
                "values": [
                    {"type": "INT", "unit": "Lx"},
                ]
            }
        }

    def set(self, name, value1 = None, value2 = None):
        if name == self.POWER:
            if value1:
                # power up
                self.write(0b00000001)
                self.power_state = True

                self.set(self.MODE, self.measure_mode, self.resolution)

            else:
                # power down
                self.write(0b00000000)
                self.power_state = False

        elif name == self.MODE:
            continuously = False
            if value1:
                continuously = True

            resolution = value2

            # power on 
            self.write(0b00000001)
            self.power_state = True

            self.time_wait = 0.140
            self.resolution = resolution

            # set mode
            opecode = 0b00100000
            self.measure_mode = self.MODE_SINGLE

            if continuously:
                opecode = 0b00010000
                self.measure_mode = self.MODE_CONTINUOUSLY

            if resolution == self.RESOLUTION_HIGH_2:
                opecode += 0b0001
            
            elif resolution == self.RESOLUTION_HIGH:
                opecode += 0b0000

            elif resolution == self.RESOLUTION_LOW:
                opecode += 0b0011
                self.time_wait = 0.025

            self.time = monotonic() + self.time_wait
            self.write(opecode)

    def get(self, name):
        if name == self.POWER:
            if self.power_state:
                return 1
            else:
                return 0

        elif name == self.MODE:
            return [self.measure_mode, self.resolution]

    def read_and_convert(self):
        self.buffer[0] = 0
        self.buffer[1] = 0

        with self.i2c_device as i2c:
            i2c.readinto(self.buffer)

        measured_lux = ((self.buffer[0] << 8) + self.buffer[1]) / 1.2
        if self.resolution == self.RESOLUTION_HIGH_2:
            measured_lux = measured_lux / 2

        self.time = monotonic() + self.time_wait

        return measured_lux

    def get_value(self):
        if not self.power_state or self.measure_mode is None:
            return None

        if self.time:
            if monotonic() < self.time:
                return None

            self.time = 0

        if self.measure_mode == self.MODE_SINGLE:
            self.measure_mode = None

        return self.read_and_convert()
