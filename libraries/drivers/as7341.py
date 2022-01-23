# SPDX-FileCopyrightText: 2022 Petr Vavrin <pvavrin@gmail.com>
# SPDX-FileCopyrightText: Copyright (c) 2022 Petr Vavrin
#
# SPDX-License-Identifier: MIT
#
# i2c_bit and i2c_bits is modified version from Adafruit_CircuitPython_Register
#           https://github.com/adafruit/Adafruit_CircuitPython_Register
#           2016 Scott Shawcroft for Adafruit Industries
#

"""
# Usage:

from as7341 import AS7341
sensor = AS7341(i2c)

sensor.set(sensor.LED_CONTROL, True)
sensor.set(sensor.LED_STATE, True)
sensor.set(sensor.LED_DRIVE, 4)

while True:
    spectral = sensor.get_value()

    if spectral:
        print ('spectral', spectral)
"""

import adafruit_bus_device.i2c_device as i2c_device

try:
    import struct
except ImportError:
    import ustruct as struct

class AS7341:
    GAIN = "gain"
    TIMING = "timing"
    LED_CONTROL = "led_control"
    LED_STATE = "led_state"
    LED_DRIVE = "led_drive"

    register_f1_f4 = [
        0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x50, 0x00,
        0x00, 0x00, 0x20, 0x04, 0x00, 0x30, 0x01, 0x50, 0x00, 0x06
    ]
    register_f5_f8 = [
        0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03, 0x50, 0x10,
        0x03, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x50, 0x00, 0x06
    ]

    step = 0
    reads = None

    def __init__(self, i2c_bus, address=0x39):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        self.register_bank(True)

        # ID / Part number Identification
        _device_id = self.i2c_bits(0x92, 1, 2, 6)

        if not _device_id in [0b001001]:
            raise RuntimeError("WRONG_DEVICE_ID")

        # PON = 1 / Power ON / AS7341 enabled
        self.i2c_bit(0x80, 0, 1)

        self.set(self.TIMING, 100, 999)
        self.set(self.GAIN, 10)

    def get_config(self):
        return {
            "name": "AS7341",
            "type": "optical",
            "direction": "I",
            "description": "11-Channel Spectral Color Sensor [415nm, 445nm, 480nm, 515nm, 555nm, 590nm, 630nm, 680nm, CLEAR, NIR]",
            "controls": {
                "gain": {
                    "mode": "RW",
                    "values": [
                        {"type": "INT", "min": 0, "max": 10}
                    ],
                    "description": "Spectral engines gain setting"
                },
                "timing": {
                    "mode": "RW",
                    "values": [
                        {"type": "INT", "min": 0, "max": 255},
                        {"type": "INT", "min": 0, "max": 65535}
                    ],
                    "description": "ADC Timing Configuration / Integration Time / ATIME, ASTEP"
                },
                "led_control": {
                    "mode": "RW",
                    "values": [
                        {"type": "INT", "min": 0, "max": 1}
                    ],
                    "description": "Enable for controls LED connected to pin LDR"
                },
                "led_state": {
                    "mode": "RW",
                    "values": [
                        {"type": "INT", "min": 0, "max": 1}
                    ],
                    "description": "External LED connected to pin LDR [on/off]"
                },
                "led_drive": {
                    "mode": "RW",
                    "values": [
                        {"type": "INT", "min": 0, "max": 10}
                    ],
                    "description": "LED driving strength"
                },
            },
            "value": {
                "mode": "R",
                "values": [
                    {"type": "INT", "alias": "415nm"},
                    {"type": "INT", "alias": "445nm"},
                    {"type": "INT", "alias": "480nm"},
                    {"type": "INT", "alias": "515nm"},
                    {"type": "INT", "alias": "555nm"},
                    {"type": "INT", "alias": "590nm"},
                    {"type": "INT", "alias": "630nm"},
                    {"type": "INT", "alias": "680nm"},
                    {"type": "INT", "alias": "clear"},
                    {"type": "INT", "alias": "nir"}
                ]
            }
        }

    def register_bank(self, high):
        if high:
            self.i2c_bit(0xA9, 4, 0)
        else:
            self.i2c_bit(0xA9, 4, 1)

    def set(self, name, value1 = None, value2 = None):
        if name == self.GAIN:
            self.register_bank(True)

            if not 0 <= value1 <= 10:
                raise ValueError("WRONG_GAIN")

            # AGAIN / Spectral engines gain setting [0 = 0.5x, 1 = 1x, 2 = 2x, 3 = 4x, 4 = 8x, ..., 10 = 512x]
            self.i2c_bits(0xAA, 1, 0, 8, value1)

        elif name == self.TIMING:
            # ADC Timing Configuration / Integration Time
            # tint = (ATIME + 1) * (ASTEP + 1) * 2.78us
            if not 0 <= value1 <= 255:
                raise ValueError("WRONG_ATIME")
            
            if not 0 <= value2 <= 65535:
                raise ValueError("WRONG_ASTEP")

            self.register_bank(True)

            # ATIME / Integration time - 0 to 255
            self.i2c_bits(0x81, 1, 0, 8, value1)

            # ASTEP / Integration time step size - 0 to 65535 - lower byte
            self.i2c_bits(0xCA, 1, 0, 8, value2 & 0xFF)

            # ASTEP / Integration time step size - 0 to 65535 - upper byte
            self.i2c_bits(0xCB, 1, 0, 8, (value2 >> 8) & 0xFF)

        elif name == self.LED_CONTROL:
            self.register_bank(False)

            if value1:
                # LED_SEL = 1 / LED control - Register LED controls LED connected to pin LDR
                self.i2c_bit(0x70, 3, 1)
            else:
                # LED_SEL = 0 / LED control - External LED not controlled by AS7341
                self.i2c_bit(0x70, 3, 0)

        elif name == self.LED_STATE:
            self.register_bank(False)

            if value1:
                # LED_ACT = 1 / LED control - External LED connected to pin LDR on
                self.i2c_bit(0x74, 7, 1)
            else:
                # LED_ACT = 0 / LED control - External LED connected to pin LDR off
                self.i2c_bit(0x74, 7, 0)

        elif name == self.LED_DRIVE:
            self.register_bank(False)

            if not 0 <= value1 <= 10:
                raise ValueError("WRONG_LED_DRIVE")

            # LED_DRIVE / LED driving strength [0x00 = 4mA, 0x01 = 6mA, ..., 0x3F = 258mA]
            self.i2c_bits(0x74, 1, 0, 7, value1)

        else:
            raise ValueError("WRONG_NAME")

    def get(self, name):
        if name == self.GAIN:
            self.register_bank(True)

            # AGAIN / Spectral engines gain setting [0 = 0.5x, 1 = 1x, 2 = 2x, 3 = 4x, 4 = 8x, ..., 10 = 512x]
            return self.i2c_bits(0xAA, 1, 0, 8)

        elif name == self.TIMING:
            self.register_bank(True)
            # ADC Timing Configuration / Integration Time

            # ATIME / Integration time - 0 to 255
            atime = self.i2c_bits(0x81, 1, 0, 8)

            # ASTEP / Integration time step size - 0 to 65535
            astep = (self.i2c_bits(0xCB, 1, 0, 8) << 8) + self.i2c_bits(0xCA, 1, 0, 8)

            return atime, astep

        elif name == self.LED_CONTROL:
            self.register_bank(False)

            # LED_SEL / LED control - Register LED controls LED connected to pin LDR
            return self.i2c_bit(0x70, 3)

        elif name == self.LED_STATE:
            self.register_bank(False)

            # LED_ACT / LED control - External LED connected to pin LDR on
            self.i2c_bit(0x74, 7)

        elif name == self.LED_DRIVE:
            self.register_bank(False)

            # LED_DRIVE / LED driving strength [0x00 = 4mA, 0x01 = 6mA, ..., 0x3F = 258mA]
            return self.i2c_bits(0x74, 1, 0, 7)

        else:
            raise ValueError("WRONG_NAME")

    def step_write_registers(self, register):
        # SP_EN = 0 / Spectral Measurement Disabled
        self.i2c_bit(0x80, 1, 0)

        # SMUX_CMD = 2 / Write SMUX configuration from RAM to SMUX chain
        self.i2c_bits(0xAF, 1, 3, 2, 2)

        # Write new configuration to all the 20 registers
        self.write_registers(register)

        # SMUXEN = 1 / Starts SMUX command
        self.i2c_bit(0x80, 4, 1)

    def step_measure_color(self):
        # SMUXEN / Wait until SMUX command finish
        if self.i2c_bit(0x80, 4) == 0:
            # SP_EN = 1 / Spectral Measurement Enabled
            self.i2c_bit(0x80, 1, 1)
            return True

    def step_wait_for_data(self):
        # AVALID / Wait until Spectral Valid state
        if self.i2c_bit(0xA3, 6) == 1:
            return True

    def step_read_result(self):
        buf = bytearray(14)
        buf[0] = 0x94

        with self.i2c_device as i2c:
            i2c.write_then_readinto(buf, buf, out_end=1, in_start=1)
            adc_reads = struct.unpack_from('<BHHHHHH', memoryview(buf)[1:])
            return adc_reads[1:]

    def get_value(self):
        self.register_bank(True)

        if self.step == 7:
            self.step = 0

        if self.step == 0:
            self.reads = None
            self.step_write_registers(self.register_f1_f4)

        elif self.step == 1:
            if not self.step_measure_color():
                return

        elif self.step == 2:
            if not self.step_wait_for_data():
                return

        elif self.step == 3:
            self.reads = (self.step_read_result())[:-2]

        elif self.step == 4:
            self.step_write_registers(self.register_f5_f8)

        elif self.step == 5:
            if not self.step_measure_color():
                return

        elif self.step == 6:
            if not self.step_wait_for_data():
                return

            self.step = 0
            result = self.reads + self.step_read_result()
            self.reads = None
            return result

        self.step += 1
        return None

    def i2c_bit(self, register_address, bit, value = None):
        return self.i2c_bits(register_address, 1, bit, 1, value)

    def i2c_bits(self, register_address, register_width, lowest_bit, num_bits, value = None):
        bit_mask = ((1 << num_bits) - 1) << lowest_bit

        if bit_mask >= 1 << (register_width * 8):
            raise RuntimeError("MORE_BITS_THAN_REGISTER_SIZE")

        buffer = bytearray(1 + register_width)
        buffer[0] = register_address

        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer, buffer, out_end=1, in_start=1)

            reg = 0
            order = range(len(buffer) - 1, 0, -1)
            for i in order:
                reg = (reg << 8) | buffer[i]

            if value is None:
                return (reg & bit_mask) >> lowest_bit

            else:
                value <<= lowest_bit
                reg &= ~bit_mask  # mask off the bits we're about to change
                reg |= value  # then or in our new value

                for i in reversed(order):
                    buffer[i] = reg & 0xFF
                    reg >>= 8

                i2c.write(buffer)

    def write_registers(self, registers):
        with self.i2c_device as i2c:
            for addr, data in enumerate(registers):
                i2c.write(bytearray([addr, data]))
