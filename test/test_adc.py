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

import sys
sys.path.append("..")
from peripherals.spi import *
from peripherals.i2c import *
from peripherals.lmh6518 import *
from peripherals.mcp4728 import *
from peripherals.lmk61e2 import *
from peripherals.trigger import *
from peripherals.had1511_adc import *
from peripherals.zl30250 import *

from test_i2c import *

# Acronyms:
# AFE: Analog Front-End.
# ADC: Analog to Digital Converter.
# PGA: Programmable Gain Amplifier.

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

# AFE Configure ------------------------------------------------------------------------------------

def afe_configure(host, port, channel, coupling, attenuation):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    print("Analog Front-End (AFE) Configuration...")
    print("---------------------------------------")

    # LDO.
    def configure_ldo(enable):
        control_value  = bus.regs.frontend_control.read()
        control_value &= ~(     1 * AFE_CONTROL_LDO_EN)
        control_value |=  (enable * AFE_CONTROL_LDO_EN)
        bus.regs.frontend_control.write(control_value)

    print("- Enabling LDO.")
    configure_ldo(1)

    # Coupling.
    def configure_coupling(channel, coupling):
        value = {
            "AC" : AFE_AC_COUPLING,
            "DC" : AFE_DC_COUPLING,
        }[coupling.upper()]
        control_value  = bus.regs.frontend_control.read()
        control_value &= ~((    1 << channel) * AFE_CONTROL_COUPLING)
        control_value |=  ((value << channel) * AFE_CONTROL_COUPLING)
        bus.regs.frontend_control.write(control_value)

    print(f"- Coupling: {coupling}.")
    configure_coupling(channel, coupling)

    # Attenuation.
    def configure_attenuation(channel, attenuation):
        value = {
            "1X"  :  AFE_1X_ATTENUATION,
            "10X" : AFE_10X_ATTENUATION,
        }[attenuation.upper()]
        control_value  = bus.regs.frontend_control.read()
        control_value &= ~((    1 << channel) * AFE_CONTROL_ATTENUATION)
        control_value |=  ((value << channel) * AFE_CONTROL_ATTENUATION)
        bus.regs.frontend_control.write(control_value)

    print(f"- Attenuation: {attenuation}.")
    configure_attenuation(channel, attenuation)

    print("")

    bus.close()

# PGA Configure ------------------------------------------------------------------------------------

def pga_configure(host, port, channel, preamp_db, atten_db, bw_mhz, offset):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    print("Programmable Gain Amplifier (PGA) Configuration...")
    print("--------------------------------------------------")

    # Programmable Gain Amplifier.
    print(f"- Preamp      : +{preamp_db}dB.")
    print(f"- Attenuation : -{atten_db}dB.")
    print(f"- Bandwidth   : {bw_mhz}MHz.")
    print(f"- Offset      : {offset}.")

    # LMH6518.
    print("- Configuring LMH6518 (SPI)...")
    pga = LMH6518Driver(bus=bus, name="frontend_spi")
    pga.set(channel, preamp_db, atten_db, bw_mhz)

    # MCP4728.
    print("- Configuring MCP4728 (I2C)...")
    dac = MCP4728Driver(bus=bus, name="i2c", addr=MCP4728_I2C_ADDR)
    dac.set_ch(n=channel, value=offset)

    print("")

    bus.close()

# ADC Configure ------------------------------------------------------------------------------------

def adc_configure(host, port, channel, mode, downsampling):
    assert mode in ["capture", "ramp"]
    bus = RemoteClient(host=host, port=port)
    bus.open()

    print("Analog to Digital Converter (ADC) Configuration...")
    print("--------------------------------------------------")

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

    print("- Enabling ZL30250 PLL.")
    configure_pll(1)

    print("- Configuring ZL30250 PLL (I2C)...")
    zl30250 = ZL30250Driver(bus=bus, name="i2c", addr=ZL30250_I2C_ADDR)
    zl30250.init(config=ZL30250_I2C_CONF, debug=True)

    # PWR_DOWN.
    def configure_pwr_down(enable):
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_PWR_DOWN)
        control_value |=  (enable * ADC_CONTROL_PWR_DOWN)
        bus.regs.adc_control.write(control_value)

    print("- Enabling HAD1511.")
    configure_pwr_down(0)

    # RST.
    def configure_rst(enable):
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_RST)
        control_value |=  (enable * ADC_CONTROL_RST)
        bus.regs.adc_control.write(control_value)

    print("- Reseting HAD1511.")
    configure_rst(0)


    # HAD1511.
    print("- Configuring HAD1511 (SPI)...")
    spi = SPIDriver(bus=bus, name="frontend_spi")
    adc = HAD1511ADCDriver(bus, spi, n=0)
    adc.reset()
    adc.downsampling.write(downsampling)
    adc.data_mode(n=channel)
    if mode == "ramp":
        adc.enable_ramp_pattern()

    print("- Checking HAD1511 Samplerate...")
    msps = adc.get_samplerate(duration=0.5)
    print(f"{msps/1e6:3.2f} MSPS")

    print("- Checking HAD1511<->FPGA Synchronization...")
    bus.regs.adc_had1511_control.write(HAD1511_CORE_CONTROL_DELAY_RST)
    bitslip_count_last = bus.regs.adc_had1511_bitslip_count.read()
    for d in range(32):
        bus.regs.adc_had1511_control.write(HAD1511_CORE_CONTROL_DELAY_INC)
        time.sleep(0.01)
        bitslip_count = bus.regs.adc_had1511_bitslip_count.read()
        bitslip_diff  = (bitslip_count - bitslip_count_last) # FIXME: Handle rollover.
        bitslip_count_last = bitslip_count
        print("1" if bitslip_diff == 0 else "0", end="")
        sys.stdout.flush()
    print("")
    bus.regs.adc_had1511_control.write(HAD1511_CORE_CONTROL_DELAY_RST)

    trigger = TriggerDriver(bus)
    trigger.reset()
    trigger.enable()

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Frontend test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    parser.add_argument("--channels",     default="0",         help="ADC Channels: 0 (default), 1, 2, 3 or combinations (01, 23, 0123).")
    parser.add_argument("--mode",         default="capture",   help="ADC Mode: capture (default), ramp.")
    parser.add_argument("--downsampling", default=1, type=int, help="ADC DownSampling Ratio (default=1).")

    parser.add_argument("--afe-coupling",    default="AC", help="AFE Coupling: AC (default) or DC.")
    parser.add_argument("--afe-attenuation", default="1X", help="AFE Attenuation: 1X (default) or 10X.")

    parser.add_argument("--pga-preamp",  default="10",   help="PGA Preamp Gain (dB), 10 or 30.")
    parser.add_argument("--pga-atten",   default="10",   help="PGA Attenuation (dB), 0 to 20 (with 2 increment).")
    parser.add_argument("--pga-bw",      default="full", help="PGA Bandwidth (MHz), 20, 100, 200, full.")
    parser.add_argument("--pga-offset",  default="0",    help="PGA Offset (0 to 4095).")

    args = parser.parse_args()

    # Parameters.
    host = args.host
    port = int(args.port, 0)
    assert len(args.channels) == 1 # FIXME: Allow multiple channels.
    channel      = int(args.channels, 0)
    mode         = args.mode
    downsampling = args.downsampling

    # Analog Frontend-Configuration.
    afe_configure(host=host, port=port,
        channel     = channel,
        coupling    = args.afe_coupling,
        attenuation = args.afe_attenuation,
    )

    # PGA configuration.
    preamp_db = int(args.pga_preamp, 0)
    atten_db  = int(args.pga_atten,  0)
    if args.pga_bw == "full":
        bw_mhz = 900
    else:
        bw_mhz = int(args.pga_bw, 0)
    offset = int(args.pga_offset) # FIXME: in V/div?
    pga_configure(host=host, port=port,
        channel   = channel,
        preamp_db = preamp_db,
        atten_db  = atten_db,
        bw_mhz    = bw_mhz,
        offset    = offset,
    )

    # ADC Configuration
    adc_configure(host=host, port=port,
        channel      = channel,
        mode         = mode,
        downsampling = downsampling,
    )

if __name__ == "__main__":
    main()
