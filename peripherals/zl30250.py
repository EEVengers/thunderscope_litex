#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Aleksa Bjelogrlic <aleksa@eevengers.com>
# Copyright (c) 2023 John Simons <john@totalitee.nl>

# SPDX-License-Identifier: BSD-2-Clause

import sys

from .i2c import *

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

ZL30250_I2C_ADDR = 0xD8
ZL30250_I2C_WRITE_COMMAND = 0x02
ZL30250_I2C_CONF = {
    # From configuration tool (Reg : Value).
    0x0423 : 0x08,
    0x0003 : 0x01,
    0x0004 : 0x02,
    0x0005 : 0x21,
    0x0007 : 0x01,
    0x0100 : 0x42,
    0x0101 : 0x00,
    0x0102 : 0x01,
    0x0106 : 0x00,
    0x0107 : 0x00,
    0x0108 : 0x00,
    0x0109 : 0x00,
    0x010A : 0x20,
    0x010B : 0x03,
    0x0121 : 0x60,
    0x0127 : 0x90,
    0x0141 : 0x00,
    0x0142 : 0x00,
    0x0143 : 0x00,
    0x0144 : 0x00,
    0x0145 : 0xA0,
    0x0153 : 0x00,
    0x0154 : 0x50,
    0x0155 : 0xCE,
    0x0180 : 0x00,
    0x0200 : 0x80,
    0x0201 : 0x05,
    0x0250 : 0x80,
    0x0251 : 0x02,
    0x0430 : 0x0C,
    0x0430 : 0x00,
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A (Directly reusing LiteX's bitbang I2C core).

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class ZL30250Driver:
    def __init__(self, bus, name, addr):
        self.i2c  = I2CDriver(bus=bus, name=name)
        self.addr = addr

    def init(self, config, debug=True):
        for reg, value in config.items():
            if debug:
                print(f"0x{reg:02x}", end="")
            ack = 0
            while (not ack):
                if debug:
                    print(".", end="")
                sys.stdout.flush()
                self.i2c.start_cond()
                ack =  self.i2c.write(self.addr)
                ack =  self.i2c.write(ZL30250_I2C_WRITE_COMMAND)
                ack &= self.i2c.write(reg >> 8)
                ack &= self.i2c.write(reg & 0xFF)
                ack &= self.i2c.write(value)
                self.i2c.stop_cond()
            if debug:
                print("")