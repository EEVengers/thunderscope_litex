#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from .spi import *

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

PREAMP_FIELD_VALUES = {
#   dB : Binary value.
    10 : 0b0,
    30 : 0b1,
}

ATTEN_FIELD_VALUES = {
#   dB : Binary value.
    0  : 0b0000,
    2  : 0b0001,
    4  : 0b0010,
    6  : 0b0011,
    8  : 0b0100,
    10 : 0b0101,
    12 : 0b0110,
    14 : 0b0111,
    16 : 0b1000,
    18 : 0b1001,
    20 : 0b1010,
}

BW_FIELD_VALUES = {
#    MHz : Binary value.
     900 : 0b000,
      20 : 0b001,
     100 : 0b010,
     200 : 0b011,
     350 : 0b100,
     650 : 0b100,
     750 : 0b110,
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A (Directly reusing LiteX's SPIMaster core).

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class LMH6518Driver:
    def __init__(self, bus, name):
        self.spi = SPIDriver(bus=bus, name=name)

    def set(self, channel, preamp_db=10, atten_db=10, bw_mhz=900):
        # Compute Configuration Fields.
        preamp_field = PREAMP_FIELD_VALUES[preamp_db]
        atten_field  = ATTEN_FIELD_VALUES[atten_db]
        bw_field     = BW_FIELD_VALUES[bw_mhz]

        # Prepare SPI Data.
        cmd_field = 0
        dat_field = 0
        dat_field |= (           1 << 10) # Aux Hi-Z.
        dat_field |= (bw_field     <<  6) # Filter.
        dat_field |= (preamp_field <<  4) # Preamp.
        dat_field |= (atten_field  <<  0) # Attenuation.
        spi_data  = []
        spi_data.append(cmd_field)
        spi_data.append((dat_field >> 8) & 0xff)
        spi_data.append((dat_field >> 0) & 0xff)
        self.spi.write(cs=channel+1, data=spi_data)
