#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
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

MCP4728_I2C_ADDR = 0x60

MCP4728_GENERAL_CALL_COMMAND = 0x00
MCP4728_GENERAL_CALL_RESET   = 0x06
MCP4728_WRITE_DAC_REGISTER   = 0x40


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A (Directly reusing LiteX's bitbang I2C core).

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class MCP4728Driver:
    def __init__(self, bus, name, addr):
        self.i2c  = I2CDriver(bus=bus, name=name)
        self.addr = addr

    def rst(self):
        self.i2c.start_cond()
        self.i2c.write(I2C_W_ADDR(self.addr))
        self.i2c.write(MCP4728_GENERAL_CALL_COMMAND)
        self.i2c.write(MCP4728_GENERAL_CALL_RESET)
        self.i2c.stop_cond()

    def set_ch(self, n, value):
        assert n < 4
        self.i2c.start_cond()
        self.i2c.write(I2C_W_ADDR(self.addr))
        self.i2c.write(MCP4728_WRITE_DAC_REGISTER + (n << 1))
        self.i2c.write((value >> 8) & 0x0f)
        self.i2c.write((value >> 0) & 0xff)
        self.i2c.stop_cond()
