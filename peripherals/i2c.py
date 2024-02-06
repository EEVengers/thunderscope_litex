#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import time
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

I2C_SCL    = 0x01
I2C_SDAOE  = 0x02
I2C_SDAOUT = 0x04
I2C_SDAIN  = 0x01

I2C_DELAY = 1
I2C_WRITE = 0
I2C_READ  = 1

def I2C_W_ADDR(addr):
    return (addr & 0x7f) << 1

def I2C_R_ADDR(addr):
    return ((addr & 0x7f) << 1) | 1

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A (Directly reusing LiteX's bitbang I2C core).

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class I2CDriver:
    def __init__(self, bus, name):
        self.w = getattr(bus.regs, f"{name}_w")
        self.r = getattr(bus.regs, f"{name}_r")

        self.started = 0

        # Wait for I2C bus to be ready.
        self.w.write(I2C_SCL)
        while(not (self.r.read() & I2C_SDAIN)):
            time.sleep(1e-3)

    # Bit-Banging functions (Inspired from from http://en.wikipedia.org/wiki/I2c).
    def read_bit(self):
        # Let the Slave drive data.
        self.w.write(0)
        self.w.write(I2C_SCL)
        bit = (self.r.read() & I2C_SDAIN)
        self.w.write(0)
        return bit

    def write_bit(self, bit):
        if bit:
            self.w.write(I2C_SDAOE| I2C_SDAOUT)
        else:
            self.w.write(I2C_SDAOE)
        # Clock stretching.
        self.w.write(self.w.read() | I2C_SCL);
        self.w.write(self.w.read() & ~I2C_SCL);

    def start_cond(self):
        if self.started:
            # Set SDA to 1.
            self.w.write(I2C_SDAOE| I2C_SDAOUT);
            self.w.write(self.w.read() | I2C_SCL);
        # SCL is high, set SDA from 1 to 0.
        self.w.write(I2C_SDAOE| I2C_SCL)
        self.w.write(I2C_SDAOE)
        self.started = 1

    def stop_cond(self):
        # Set SDA to 0.
        self.w.write(I2C_SDAOE)
        # Clock stretching.
        self.w.write(I2C_SDAOE| I2C_SCL)
        # SCL is high, set SDA from 0 to 1.
        self.w.write(I2C_SCL)
        self.started = 0

    # Byte functions.
    def write(self, byte):
        for bit in range(8):
            self.write_bit(byte & 0x80)
            byte <<= 1
        ack = not self.read_bit()
        return ack

    def read(self, ack):
        byte = 0
        for bit in range(8):
            byte <<= 1
            byte |= self.read_bit()
        self.write(not ack)
        return byte

    # Polling function.
    def poll(self, addr):
        self.start_cond()
        ack  = self.write(I2C_W_ADDR(addr))
        ack |= self.write(I2C_R_ADDR(addr))
        self.stop_cond()
        return ack
