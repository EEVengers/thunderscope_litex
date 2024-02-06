#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import time
import argparse

from litex import RemoteClient

sys.path.append("..")
from peripherals.i2c import *

from peripherals.zl30250 import *

# AFE Frontend Constants ---------------------------------------------------------------------------

AFE_CONTROL_LDO_EN      = (1 <<  0)
AFE_CONTROL_COUPLING    = (1 <<  8)
AFE_CONTROL_ATTENUATION = (1 << 16)

AFE_STATUS_LDO_PWR_GOOD = (1 <<  0)

AFE_AC_COUPLING = 0
AFE_DC_COUPLING = 1

AFE_1X_ATTENUATION  = 0
AFE_10X_ATTENUATION = 1

# ADC Constants ------------------------------------------------------------------------------------

ADC_CONTROL_LDO_EN   = (1 <<  0)
ADC_CONTROL_PLL_EN   = (1 <<  1)
ADC_CONTROL_RST      = (1 <<  2)
ADC_CONTROL_PWR_DOWN = (1 <<  3)

ADC_STATUS_LDO_PWR_GOOD = (1 <<  0)

# I2C Scan -----------------------------------------------------------------------------------------

def i2c_scan(host, port):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    # LDO.
    def configure_frontend_ldo(enable):
        control_value  = bus.regs.frontend_control.read()
        control_value &= ~(     1 * AFE_CONTROL_LDO_EN)
        control_value |=  (enable * AFE_CONTROL_LDO_EN)
        bus.regs.frontend_control.write(control_value)

    print("- Enabling frontend LDO.")
    configure_frontend_ldo(1)

    # LDO.
    def configure_ldo(enable):
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_LDO_EN)
        control_value |=  (enable * ADC_CONTROL_LDO_EN)
        bus.regs.adc_control.write(control_value)

    print("- Enabling ADC LDO.")
    configure_ldo(1)

    # PLL.
    def configure_pll(enable):
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_PLL_EN)
        control_value |=  (enable * ADC_CONTROL_PLL_EN)
        bus.regs.adc_control.write(control_value)

    print("- Disabling ZL30250 LDO.")
    configure_pll(0)
    time.sleep(1)

    print("- Enabling ZL30250 PLL.")
    configure_pll(1)
    
    #print("- Configuring ZL30250 PLL (I2C)...")
    #zl30250 = ZL30250Driver(bus=bus, name="i2c", addr=ZL30250_I2C_ADDR)
    #zl30250.read(0x0001)

    i2c = I2CDriver(bus=bus, name="i2c")

    print("      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f", end="");
    sys.stdout.flush()
    for addr in range(0, 0x80):
        if (addr % 0x10) == 0:
            print(f"\n0x{addr:02x}", end="")
        if i2c.poll(addr):
            print(f" {addr:02x}", end="")
        else:
            print(f" --", end="")
        sys.stdout.flush()
    print("")

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="I2C test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    parser.add_argument("--scan",         action="store_true", help="Scan I2C Bus.")
    args = parser.parse_args()

    host = args.host
    port = int(args.port, 0)

    if args.scan:
        i2c_scan(host=host, port=port)

if __name__ == "__main__":
    main()
