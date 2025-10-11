#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

import shutil, os
import subprocess

from migen import *

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.xilinx import Xilinx7SeriesPlatform
from litex.build.openfpgaloader import OpenFPGALoader

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import *
from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.pwm import PWM

from litei2c import LiteI2C

from litespi.modules import MX25U6435E
from litespi.opcodes import SpiNorFlashOpCodes as Codes
from litespi.spi_nor_flash_module import SpiNorFlashModule
from litespi.ids import SpiNorFlashManufacturerIDs

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

from litescope import LiteScopeAnalyzer

from peripherals.windowRemapper import WindowRemapper
from peripherals.hmcad1520_adc import HMCAD1520ADC
from peripherals.trigger import Trigger


def get_commit_hash_string():
    # Get the hash for the current commit
    commit_hash = subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode('ascii')
    # Check if the repo is dirty
    repo_dirty = bool(subprocess.check_output(['git', 'status', '--porcelain']).decode('ascii').strip())
    return commit_hash, repo_dirty

# IOs ----------------------------------------------------------------------------------------------
# Trentz A100T/A200T Module
a7_484_io = [
     # Main system clock. 
    ("clk50", 0,
        Subsignal("p", Pins("H4"), IOStandard("DIFF_SSTL15")),
        Subsignal("n", Pins("G4"), IOStandard("DIFF_SSTL15")),
    ),

    # Leds.
    # -----
    ("user_led_n", 0, Pins("T21"), IOStandard("LVCMOS33")), # Green.

    # SPI Flash.
    # ----------
    ("spiflash4x", 0,
        Subsignal("cs_n", Pins("T19")),
        # Subsignal("clk",  Pins("L12")),
        Subsignal("dq",   Pins("P22 R22 P21 R21")),
        IOStandard("LVCMOS33")
    ),

    # PCIe / Gen2 X4.
    # ---------------
    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("V14"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("F10")),
        Subsignal("clk_n", Pins("E10")),
        Subsignal("rx_p",  Pins("D9 B10 D11 B8")),
        Subsignal("rx_n",  Pins("C9 A10 C11 A8")),
        Subsignal("tx_p",  Pins("D7 B6 D5 B4")),
        Subsignal("tx_n",  Pins("C7 A6 C5 A4")),
    ),

    # Frontend.
    # ---------

    # Probe Compensation.
    ("fe_probe_compensation", 0, Pins("N20"), IOStandard("LVCMOS33")),

    # Control / Status.
    ("fe_control", 0,
        Subsignal("fe_en",       Pins("J21"), IOStandard("LVCMOS33")), # TPS7A9101/LDO & LM27761 Enable.
        Subsignal("coupling",    Pins("N18 H19 K19 H20"), IOStandard("LVCMOS33")),
        Subsignal("attenuation", Pins("N19 J19 K18 G20"), IOStandard("LVCMOS33")),
        Subsignal("term",        Pins("M18 L21 L19 H22"), IOStandard("LVCMOS33"))
    ),

    # SPI
    ("main_spi", 0,
        Subsignal("clk",  Pins("K21")),
        Subsignal("cs_n", Pins("L18 M21 L20 J22 K13")),
        Subsignal("mosi", Pins("K22")),
        IOStandard("LVCMOS33"),
    ),

    # I2C bus.
    # --------
    ("i2c", 0,
        Subsignal("sda", Pins("J14")),
        Subsignal("scl", Pins("H14")),
        IOStandard("LVCMOS33"),
    ),

    # ADC / HMCAD1511.
    # ----------------

    # Control / Status.
    ("adc_control", 0,
        Subsignal("acq_en", Pins("J20")), # TPS7A9101/LDO Enable.
        Subsignal("osc_oe", Pins("K14")), # LMK61E2/PLL Output Enable.
        IOStandard("LVCMOS33"),
    ),
    # Datapath.
    ("adc_data", 0,
        Subsignal("lclk_p", Pins("C18")), # Bitclock.
        Subsignal("lclk_n", Pins("C19")),
        Subsignal("fclk_p", Pins("D17")), # Frameclock.
        Subsignal("fclk_n", Pins("C17")),
        # Lane:                D1A D1B D2A D2B D3A D3B D4A D4B
        # Lanes polarity:       X   X       X   X   X   X   X      # (X=Inverted).
        Subsignal("d_p", Pins("A15 B15 B17 A13 F16 D14 E13 F13")), # Data.
        Subsignal("d_n", Pins("A16 B16 B18 A14 E17 D15 E14 F14")),
        IOStandard("RSDS_25"),
        Misc("DIFF_TERM=TRUE"),
    ),

    # SYNC
    # ----------------
    ("sync", 0, Pins("Y22"), IOStandard("LVCMOS33"))
]

# Custom xc7a50T Module
a7_325_io = [
     # Main system clock. 
    ("clk25", 0,  Pins("P4"), IOStandard("LVCMOS33")),

    # Leds.
    # -----
    ("user_led_n", 0, Pins("U17"), IOStandard("SSTL135")), # Green.

    # SPI Flash.
    # ----------
    ("spiflash4x", 0,
        Subsignal("cs_n", Pins("L15")),
        # Subsignal("clk",  Pins("E8")),
        Subsignal("dq",   Pins("K16 L17 J15 J16")),
        IOStandard("SSTL135_R")
    ),

    # PCIe / Gen2 X4.
    # ---------------
    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("L2"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B6")),
        Subsignal("clk_n", Pins("B5")),
        Subsignal("rx_p",  Pins("G4 C4 A4 E4")),
        Subsignal("rx_n",  Pins("G3 C3 A3 E3")),
        Subsignal("tx_p",  Pins("B2 D2 F2 H2")),
        Subsignal("tx_n",  Pins("B1 D1 F1 H1")),
    ),

    # Frontend.
    # ---------

    # Probe Compensation.
    ("fe_probe_compensation", 0, Pins("K3"), IOStandard("LVCMOS33")),

    # Control / Status.
    ("fe_control", 0,
        Subsignal("fe_en",       Pins("K6"), IOStandard("LVCMOS33")), # TPS7A9101/LDO & LM27761 Enable.
        Subsignal("coupling",    Pins("M6 T2 M2 N6"), IOStandard("LVCMOS33")),
        Subsignal("attenuation", Pins("L4 P1 M5 M1"), IOStandard("LVCMOS33")),
        Subsignal("term",        Pins("J4 P5 N2 P3"), IOStandard("LVCMOS33"))
    ),

    # SPI
    ("main_spi", 0,
        Subsignal("clk",  Pins("K2")),
        Subsignal("cs_n", Pins("R6 K1 R3 N1 J5")),
        Subsignal("mosi", Pins("L3")),
        IOStandard("LVCMOS33"),
    ),

    # I2C bus.
    # --------
    ("i2c", 0,
        Subsignal("sda", Pins("N4")),
        Subsignal("scl", Pins("K5")),
        IOStandard("LVCMOS33"),
    ),

    # ADC / HMCAD1511.
    # ----------------

    # Control / Status.
    ("adc_control", 0,
        Subsignal("acq_en", Pins("M4")), # TPS7A9101/LDO Enable.
        Subsignal("osc_oe", Pins("N3")), # LMK61E2/PLL Output Enable.
        IOStandard("LVCMOS33"),
    ),
    # Datapath.
    ("adc_data", 0,
        Subsignal("lclk_p", Pins("R2")), # Bitclock.
        Subsignal("lclk_n", Pins("R1")),
        Subsignal("fclk_p", Pins("U2")), # Frameclock.
        Subsignal("fclk_n", Pins("U1")),
        # Lane:                D1A D1B D2A D2B D3A D3B D4A D4B
        # Lanes polarity:               X   X       X   X   X      # (X=Inverted).
        Subsignal("d_p", Pins("U4  V3  U7  V8  R5  T4  U6  R7")),  # Data.
        Subsignal("d_n", Pins("V4  V2  V6  V7  T5  T3  U5  T7")),
        IOStandard("RSDS_25"),
        Misc("DIFF_TERM=FALSE"),
    ),

    # SYNC
    # ----------------
    ("sync", 0, Pins("P6"), IOStandard("LVCMOS33"))
]

# Thunderscope Production
a7_thunderscope_rev5 = [
    # Main system clock. 
    ("clk25", 0,  Pins("T14"), IOStandard("LVCMOS33")),

    # HW IDs
    # -------
    ("hw_id", 0, 
        Subsignal("hw_rev", Pins("T18 R18 P18")),
        Subsignal("hw_variant", Pins("K18")),
        IOStandard("LVCMOS33")),

    # Leds.
    # -----
    ("user_led_n", 0, Pins("T15 R13 U14"), IOStandard("LVCMOS33")), # Red/Green/Blue.

    # SPI Flash.
    # ----------
    ("spiflash4x", 0,
        Subsignal("cs_n", Pins("L15")),
        # Subsignal("clk",  Pins("E8")),
        Subsignal("dq",   Pins("K16 L17 J15 J16")),
        IOStandard("LVCMOS33")
    ),

    # PCIe / Gen2 X4.
    # ---------------
    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("U9"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B6")),
        Subsignal("clk_n", Pins("B5")),
        Subsignal("rx_p",  Pins("E4 A4 C4 G4")),
        Subsignal("rx_n",  Pins("E3 A3 C3 G3")),
        Subsignal("tx_p",  Pins("H2 F2 D2 B2")),
        Subsignal("tx_n",  Pins("H1 F1 D1 B1")),
    ),

    # Frontend.
    # ---------

    # Probe Compensation.
    ("fe_probe_compensation", 0, Pins("R15"), IOStandard("LVCMOS33")),

    # Control / Status.
    ("fe_control", 0,
        Subsignal("fe_en",       Pins("U15"), IOStandard("LVCMOS33")),
        Subsignal("fe_pg",       Pins("V16"), IOStandard("LVCMOS33")),
        Subsignal("coupling",    Pins("N16 M15 N18 V12"), IOStandard("LVCMOS33")),
        Subsignal("attenuation", Pins("N17 M16 L18 V11"), IOStandard("LVCMOS33")),
        Subsignal("term",        Pins("M14 K15 K17 V13"), IOStandard("LVCMOS33"))
    ),

    # SPI busses.
    # --------
    # Amplifier SPI
    ("main_spi", 0,
        Subsignal("clk",  Pins("N14")),
        Subsignal("cs_n", Pins("P16 M17 R17 U11")),
        Subsignal("mosi", Pins("P15")),
        IOStandard("LVCMOS33"),
    ),
    # ADC SPI
    ("adc_spi", 0,
        Subsignal("clk",  Pins("U16")),
        Subsignal("cs_n", Pins("U17")),
        Subsignal("mosi", Pins("R16")),
        IOStandard("LVCMOS33"),
    ),

    # I2C busses.
    # --------
    ("trim_i2c", 0,
        Subsignal("sda", Pins("V14")),
        Subsignal("scl", Pins("T13")),
        IOStandard("LVCMOS33"),
    ),
    ("pll_i2c", 0,
        Subsignal("sda", Pins("T12")),
        Subsignal("scl", Pins("U12")),
        IOStandard("LVCMOS33"),
    ),

    # ADC / HMCAD1511.
    # ----------------

    # Control / Status.
    ("adc_control", 0,
        Subsignal("acq_en",      Pins("T17"), IOStandard("LVCMOS33")),
        Subsignal("acq_pg",      Pins("V17"), IOStandard("LVCMOS33")),
        Subsignal("osc_oe",      Pins("P14")), # PLL RSTn.
        IOStandard("LVCMOS33"),
    ),


    # Datapath.
    ("adc_data", 0,
        Subsignal("lclk_p", Pins("E13")), # Bitclock.
        Subsignal("lclk_n", Pins("D14")),
        Subsignal("fclk_p", Pins("B12")), # Frameclock. (Inverted)
        Subsignal("fclk_n", Pins("A12")),
        # Lane:                D1A D1B D2A D2B D3A D3B D4A D4B
        # Lanes polarity:                   X                      # (X=Inverted).
        Subsignal("d_p", Pins(" B9 B10 D11 C11 A13 B14 D13 C14")), # Data.
        Subsignal("d_n", Pins(" A9 A10 C12 B11 A14 A15 C13 B15")),
        IOStandard("RSDS_25"),
        Misc("DIFF_TERM=TRUE"),
    ),

    # SYNC
    # ----------------
    ("sync", 0,
        Subsignal("in_p", Pins("D8")), 
        Subsignal("in_n", Pins("C8")), 
        Subsignal("out_p", Pins("D9")), 
        Subsignal("out_n", Pins("C9")),
        Subsignal("re_n", Pins("U10"), IOStandard("LVCMOS33")),
        Subsignal("de", Pins("V9"), IOStandard("LVCMOS33")),
        IOStandard("LVDS_25"),
        Misc("DIFF_TERM=TRUE"))
]

# Platform -----------------------------------------------------------------------------------------

class Platform(Xilinx7SeriesPlatform):
    device_list = {
        "a100t" : {"fpga": "xc7a100tfgg484-2", "io": a7_484_io, "flash": "bscan_spi_xc7a100t.bit", "multiboot_addr": 0x100_0000, "multiboot_end": 0x1B0_0000, "flash_size": 32, "cfgbvs": "VCCO", "config": "3.3"},
        "a200t" : {"fpga": "xc7a200tfbg484-2", "io": a7_484_io, "flash": "bscan_spi_xc7a200t.bit", "multiboot_addr": 0x100_0000, "multiboot_end": 0x1B0_0000, "flash_size": 32, "cfgbvs": "VCCO", "config": "3.3"},
        "a50t"  : {"fpga": "xc7a50tcsg325-2",  "io": a7_325_io, "flash": "bscan_spi_xc7a50t.bit", "multiboot_addr": 0x40_0000, "multiboot_end": 0x68_0000, "flash_size": 8, "cfgbvs": "GND", "config": "1.8"},
        "a35t"  : {"fpga": "xc7a35tcsg325-2",  "io": a7_325_io, "flash": "bscan_spi_xc7a35t.bit", "multiboot_addr": 0x40_0000, "multiboot_end": 0x68_0000, "flash_size": 8, "cfgbvs": "GND", "config": "1.8"},
        "dev"   : {"fpga": "xc7a50tcsg325-2",  "io": a7_thunderscope_rev5, "flash": "bscan_spi_xc7a60t.bit", "multiboot_addr": 0x40_0000, "multiboot_end": 0x68_0000, "flash_size": 8, "cfgbvs": "VCCO", "config": "3.3"},
        "prod"  : {"fpga": "xc7a35tcsg325-2",  "io": a7_thunderscope_rev5, "flash": "bscan_spi_xc7a60t.bit", "multiboot_addr": 0x40_0000, "multiboot_end": 0x68_0000, "flash_size": 8, "cfgbvs": "VCCO", "config": "3.3"},
    }
    def __init__(self, toolchain="vivado", variant="dev"):

        Xilinx7SeriesPlatform.__init__(self, 
                                self.device_list[variant]["fpga"],
                                self.device_list[variant]["io"],
                                toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 40 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGFALLBACK ENABLE [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            f"set_property CFGBVS {self.device_list[variant]['cfgbvs']} [current_design]",
            f"set_property CONFIG_VOLTAGE {self.device_list[variant]['config']} [current_design]"
        ]

        if self.device_list[variant]["flash_size"] > 16 :
            self.toolchain.bitstream_commands.append("set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES [current_design]")

        # Set the addresses for the Barrier Images
        barrier_a_addr = self.device_list[variant]['multiboot_addr'] - 0x400
        barrier_b_addr = self.device_list[variant]['multiboot_end']
        load_barrier_imgs = f"-loaddata \"up 0x{barrier_a_addr:08X} barrierA.bin up 0x{barrier_b_addr:08X} barrierB.bin\""

        self.toolchain.additional_commands = [
            # Non-Multiboot SPI-Flash bitstream generation.
            f"write_cfgmem -force -format bin -interface spix4 -size {self.device_list[variant]['flash_size']} -loadbit \"up 0x0 {{build_name}}.bit\" -file {{build_name}}.bin",
            # Multiboot bitstreams
            "write_bitstream -force -bin_file {build_name}_update.bit",
            f"set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x{barrier_a_addr:08X} [current_design]",
            "write_bitstream -force -bin_file {build_name}_gold.bit",
            f"write_cfgmem -force -format bin -size {self.device_list[variant]['flash_size']} -interface SPIx4 -loadbit \"up 0x00000000 {{build_name}}_gold.bit up 0x{self.device_list[variant]['multiboot_addr']:08X} {{build_name}}_update.bit\" {load_barrier_imgs} {{build_name}}_full.bin",
            f"write_cfgmem -force -format mcs -size {self.device_list[variant]['flash_size']} -interface SPIx4 -loadbit \"up 0x00000000 {{build_name}}_gold.bit up 0x{self.device_list[variant]['multiboot_addr']:08X} {{build_name}}_update.bit\" {load_barrier_imgs} {{build_name}}_full.mcs",
        ]

    def create_programmer(self, variant="dev", cable="digilent_hs2"):
        if variant == 'prod':
            return OpenFPGALoader(fpga_part="xc7a35tcsg325", cable=cable)
        elif variant == 'dev':
            return OpenFPGALoader(fpga_part="xc7a50tcsg325", cable=cable)
        elif variant == 'a50t':
            return OpenFPGALoader(fpga_part="xc7a50tcsg325_1v35", cable=cable)
        elif variant == 'a100t':
            return OpenFPGALoader(fpga_part="xc7a100tfgg484", cable=cable)
        elif variant == 'a200t':
            return OpenFPGALoader(fpga_part="xc7a200tfbg484", cable=cable)
        else:
            raise ValueError("Unknown FPGA Variant for flashing", variant)

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("adc_data:lclk_p", loose=True), 1e9/500e6)
        self.add_false_path_constraint(self.lookup_request("adc_data:lclk_p", loose=True), self.lookup_request("sys:clk", loose=True))
        

# CRG ----------------------------------------------------------------------------------------------


class CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_idelay = ClockDomain()

        # PLL.
        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        # Trenz modules have 50MHz Clock, TS Module has 25MHz Clock
        clk = platform.request("clk50", loose = True)
        if clk is not None:
            pll.register_clkin(clk, 50e6)
            platform.add_period_constraint(pll.clkin, 1e9/50e6)
        else:
            pll.register_clkin(platform.request("clk25"), 25e6)
            platform.add_period_constraint(pll.clkin, 1e9/25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, reset_buf="bufg")
        pll.create_clkout(self.cd_idelay, 200e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)
        platform.add_period_constraint(self.cd_idelay.clk, 1e9/200e6)

        # IDELAYCTRL.
        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    SoCCore.csr_map = {
        "dna": 0,
        "identifier_mem": 1,
        "pcie_phy": 2,
        "pcie_msi": 3,
        "pcie_endpoint": 4,
        "pcie_dma0": 5,
        "ctrl": 6,
        "spiflash_core": 7,
        "spiflash_phy": 8,
        "flash_adapter": 9,
        "icap": 10,
        "xadc": 11,
        "dev_status": 12,
        "adc": 13,
        "frontend": 14,
        "probe_compensation": 15,
        "i2cbus": 16,
        "spibus": 17
        # Max Offset: 31
        # Offset 32+ reserved for ota mem
    }
    SoCCore.mem_map = {
        "csr": 0x0000_0000,
        "ota": 0x0001_0000,
        "spiflash": 0x1000_0000
    }

    def __init__(self, sys_clk_freq=int(150e6),
        variant       ="dev",
        with_frontend = True,
        with_adc      = True,
        with_jtagbone = True,
        with_analyzer = False,
        **kwargs
    ):
        platform = Platform(variant=variant)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        if os.getenv("BUILD_VERSION") is not None:
            version_string = os.getenv("BUILD_VERSION")
        else:
            commit, dirty = get_commit_hash_string()
            version_string = commit[0:8]
            if dirty:
                version_string += "-dirty"
            
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = f"LitePCIe SoC on ThunderScope {variant.upper()} ({version_string})",
            ident_version = True,
        )


        # Device Status------------------------------------------------------------------------------------
        
        class DeviceStatus(Module, AutoCSR):
            def __init__(self, led_pads, hw_id_pads, sys_clk_freq):
                
                # Status LEDs
                self._leds = CSRStorage(8, description="Status LED Bits")
                
                # HW ID Pins
                self._hw_id = hw_id = CSRStatus(fields=[
                                    CSRField("hw_rev", offset=0, size=3, description="HW Revision."),
                                    CSRField("hw_variant", offset=8, size=1, description="HW Bus Variant.", values=[
                                        ("``0b0``", "PCIe"),
                                        ("``0b1``", "TB"),
                                    ]),
                                    CSRField("hw_valid", offset=9, size=1, description="HW ID is valid for this board."),
                                    CSRField("num_leds", offset=16, size=4, description="Number of LEDs available")
                                ])

                if led_pads is not None:
                    self.comb += led_pads.eq(self._leds.storage)

                if hw_id_pads is not None:
                    self.comb += [
                        hw_id.fields.hw_rev.eq(hw_id_pads.hw_rev),
                        hw_id.fields.hw_variant.eq(hw_id_pads.hw_variant),
                        hw_id.fields.hw_valid.eq(1)
                    ]
                else:
                    self.comb += [
                        hw_id.fields.hw_rev.eq(0),
                        hw_id.fields.hw_variant.eq(0),
                        hw_id.fields.hw_valid.eq(0)
                    ]

        self.submodules.dev_status = DeviceStatus(
                led_pads     = platform.request("user_led_n"),
                hw_id_pads   = platform.request("hw_id", loose=True),
                sys_clk_freq     = sys_clk_freq,
            )

        # JTAGBone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()

        # XADC -------------------------------------------------------------------------------------
        self.submodules.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.submodules.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # PCIe -------------------------------------------------------------------------------------
        self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
            data_width = 128,
            bar0_size  = 0x2_0000
        )
        self.pcie_phy.config.update({
            "Vendor_ID": "20A7",
            "Device_ID": "0101"
        })
        self.add_pcie(phy=self.pcie_phy, ndmas=1, dma_buffering_depth=1024*16,
                      max_pending_requests=4, address_width=64)


        # SPI Flash --------------------------------------------------------------------------------
        #A100T/A200T
        class S25FL256S(SpiNorFlashModule):
            manufacturer_id = SpiNorFlashManufacturerIDs.SPANSION
            device_id = 0x0219
            name = "s25fl256s"

            total_size  =   33554432   # bytes
            page_size   =        256   # bytes
            total_pages =     131072

            supported_opcodes = [
                Codes.READ_1_1_4_4B,
                Codes.PP_1_1_4_4B,
                Codes.SE_4B,
            ]
            dummy_bits = 8

        spi_flash_modules = {
            "a100t": lambda: S25FL256S(Codes.READ_1_1_4_4B, program_cmd=Codes.PP_1_1_4_4B, erase_cmd=Codes.SE_4B),
            "a200t": lambda: S25FL256S(Codes.READ_1_1_4_4B, program_cmd=Codes.PP_1_1_4_4B, erase_cmd=Codes.SE_4B),
            "a50t":  lambda: MX25U6435E(Codes.READ_1_1_4, program_cmd=Codes.PP_1_1_4),
            "a35t":  lambda: MX25U6435E(Codes.READ_1_1_4, program_cmd=Codes.PP_1_1_4),
            "dev":   lambda: MX25U6435E(Codes.READ_1_1_4, program_cmd=Codes.PP_1_1_4),
            "prod":  lambda: MX25U6435E(Codes.READ_1_1_4, program_cmd=Codes.PP_1_1_4)
        }
        self.add_spi_flash(mode="4x", module=spi_flash_modules[variant](), clk_freq=65e6,
                           rate="1:1", with_mmap=True, with_master=True, with_mmap_write="csr")

        # # QSPI Flash Adapter -----------------------------------------------------------------------
        pcie_translated = wishbone.Interface(bursting=True)
        pcie_wb = self.bus.masters["pcie_mmap"]

        self.submodules.flash_adapter = WindowRemapper(
            master = pcie_wb,
            slave=pcie_translated,
            src_regions = [SoCRegion(origin=self.mem_map.get("ota", None), size=0x1_0000)],
            dst_regions = [SoCRegion(origin=self.mem_map.get("spiflash", None), size=0x200_0000)]
        )
        self.bus.masters["pcie_mmap"] = pcie_translated

        # ICAP (For FPGA reload over PCIe) ---------------------------------------------------------
        from litex.soc.cores.icap import ICAP
        self.submodules.icap = ICAP()
        self.icap.add_reload()
        self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # Frontend / ADC ---------------------------------------------------------------------------

        # I2C Bus:
        # - Trim DAC (MCP4728 @ 0x60).
        # - PLL      (ZL30260 @ 0x74).
        # - Digi-pot (MCP4432 @ 0x2C).
        # # #
        class I2C_Busses(LiteXModule):
            bus_count = 0
            def _init_(self):
                self.bus_count = 0

            def add_bus(self, i2c_pads):
                # I2C
                self.add_module(f"i2c{self.bus_count}", LiteI2C(sys_clk_freq=sys_clk_freq, pads=i2c_pads))
                self.bus_count += 1

        self.submodules.i2cbus = i2c_group = I2C_Busses()
        i2c_pads = platform.request("i2c", loose=True)
        if i2c_pads is not None:
            i2c_group.add_bus(i2c_pads)
        
        i2c_pads = platform.request("trim_i2c", loose=True)
        if i2c_pads is not None:
            i2c_group.add_bus(i2c_pads)
        
        i2c_pads = platform.request("pll_i2c", loose=True)
        if i2c_pads is not None:
            i2c_group.add_bus(i2c_pads)

        # SPI Bus
        class SPI_Busses(LiteXModule):
            bus_count = 0
            def _init_(self):
                self.bus_count = 0

            def add_master(self, spi_pads, spi_freq, sys_freq, width):
                if not hasattr(spi_pads, "miso"):
                    spi_pads.miso = Signal()
                self.add_module(f"spi{self.bus_count}", SPIMaster(
                                        pads         = spi_pads,
                                        data_width   = width,
                                        sys_clk_freq = sys_freq,
                                        spi_clk_freq = spi_freq
                                    ))
                self.bus_count += 1

        self.submodules.spibus = spi_group = SPI_Busses()

        spi_pads = platform.request("main_spi", loose=True)
        if spi_pads is not None:
            spi_group.add_master(spi_pads, 1e6, sys_clk_freq, 24)
            
        spi_pads = platform.request("adc_spi", loose=True)
        if spi_pads is not None:
            spi_group.add_master(spi_pads, 1e6, sys_clk_freq, 24)

        # Probe Compensation.
        self.submodules.probe_compensation = PWM(
            pwm = platform.request("fe_probe_compensation"),
            default_enable = 1,
            default_width  = int(1e-3*sys_clk_freq/2),
            default_period = int(1e-3*sys_clk_freq)
        )


        # Frontend.
        if with_frontend:

            class Frontend(Module, AutoCSR):
                def __init__(self, control_pads, sys_clk_freq):
                    # Control/Status.
                    self._control = CSRStorage(fields=[
                        CSRField("fe_en", offset=0, size=1, description="Frontend LDO-Enable.", values=[
                            ("``0b0``", "LDO disabled."),
                            ("``0b1``", "LDO enabled."),
                        ]),
                        CSRField("coupling",  offset=8, size=4, description="Frontend AC/DC Coupling.", values=[
                            ("``0b0``", "AC-Coupling (one bit per channel)."),
                            ("``0b1``", "DC-Coupling (one bit per channel)."),
                        ]),
                        CSRField("attenuation",  offset=16, size=4, description="Frontend Attenuation.", values=[
                            ("``0b0``", "50X-Attenuation (one bit per channel)."),
                            ("``0b1``", " 1X-Attenuation (one bit per channel)."),
                        ]),
                        CSRField("termination",  offset=24, size=4, description="Frontend Termination.", values=[
                            ("``0b0``", "1MOhm Termination (one bit per channel)."),
                            ("``0b1``", "50Ohm Termination (one bit per channel)."),
                        ]),
                    ])
                    self._status = CSRStatus(fields=[
                        CSRField("fe_pg", offset=0, size=1, description="Frontend Power Good.", values=[
                            ("``0b0``", "LDO No Power."),
                            ("``0b1``", "LDO Power."),
                        ]),
                    ])
                    # # #

                    # Power.
                    self.comb += control_pads.fe_en.eq(self._control.fields.fe_en)

                    # Coupling.
                    self.comb += control_pads.coupling.eq(self._control.fields.coupling)

                    # Attenuation.
                    self.comb += control_pads.attenuation.eq(self._control.fields.attenuation)

                    # Termination.
                    self.comb += control_pads.term.eq(self._control.fields.termination)

                    # Frontend Power Good.
                    if hasattr(control_pads,'fe_pg'):
                        self.sync += self._status.fields.fe_pg.eq(control_pads.fe_pg)

            self.submodules.frontend = Frontend(
                control_pads     = platform.request("fe_control"),
                sys_clk_freq     = sys_clk_freq,
            )

        # ADC.
        if with_adc:

            class ADC(Module, AutoCSR):
                def __init__(self, control_pads, data_pads, sys_clk_freq,
                    data_width = 128, frame_polarity = 1, data_polarity = [1, 1, 0, 1, 1, 1, 1, 1]
                ):

                    # Control/Status.
                    self._control = CSRStorage(fields=[
                        CSRField("acq_en", offset=0, size=1, description="ADC LDO-Enable.", values=[
                            ("``0b0``", "LDO disabled."),
                            ("``0b1``", "LDO enabled."),
                        ]),
                        CSRField("osc_en", offset=1, size=1, description="ADC-PLL Output-Enable.", values=[
                            ("``0b0``", "PLL output disabled."),
                            ("``0b1``", "PLL output enabled."),
                        ]),
                        CSRField("rst", offset=2, size=1, description="ADC Reset.", values=[
                            ("``0b0``", "ADC in operational mode."),
                            ("``0b1``", "ADC in reset mode."),
                        ]),
                        CSRField("pwr_down", offset=3, size=1, description="ADC Power-Down.", values=[
                            ("``0b0``", "ADC in operational mode."),
                            ("``0b1``", "ADC in power-down mode."),
                        ]),
                    ])
                   
                    self._status = CSRStatus(fields=[
                        CSRField("acq_pg", offset=0, size=1, description="ADC Power Good.", values=[
                            ("``0b0``", "ADC No Power."),
                            ("``0b1``", "ADC Power."),
                        ]),
                    ])

                    # Data Source.
                    self.source = stream.Endpoint([("data", data_width)])

                    # # #

                    # Control-Path -----------------------------------------------------------------

                    # Control.
                    self.comb += [
                        control_pads.acq_en.eq(self._control.fields.acq_en),
                        control_pads.osc_oe.eq(self._control.fields.osc_en),
                    ]

                    # Status.
                    if hasattr(control_pads,'acq_pg'):
                        self.sync += self._status.fields.acq_pg.eq(control_pads.acq_pg)

                    # Data-Path --------------------------------------------------------------------

                    # Trigger.
                    self.submodules.trigger = Trigger()

                    # HMCAD1520.
                    self.submodules.hmcad1520 = HMCAD1520ADC(pads=data_pads,
                                                             sys_clk_freq=sys_clk_freq,
                                                             frame_polarity=frame_polarity,
                                                             lanes_polarity=data_polarity,
                                                             clock_domain="sys")

                    # Gate.
                    self.submodules.gate = stream.Gate([("data", data_width)], sink_ready_when_disabled=True)
                    self.comb += self.gate.enable.eq(self.trigger.enable)

                    # Pipeline.
                    self.submodules += stream.Pipeline(
                        self.hmcad1520,
                        self.gate,
                        self.source
                    )

            adc_polarity = {"a100t" : [1, 1, 0, 1, 1, 1, 1, 1],
                            "a200t" : [1, 1, 0, 1, 1, 1, 1, 1],
                            "a50t"  : [0, 0, 1, 1, 0, 1, 1, 1],
                            "a35t"  : [0, 0, 1, 1, 0, 1, 1, 1],
                            "dev"   : [0, 0, 0, 1, 0, 0, 0, 0],
                            "prod"  : [0, 0, 0, 1, 0, 0, 0, 0],
                            }
            frame_polarity = {"a100t" : 0,
                              "a200t" : 0,
                              "a50t"  : 0,
                              "a35t"  : 0,
                              "dev"   : 1,
                              "prod"  : 1,
                              }

            self.submodules.adc = ADC(
                control_pads = platform.request("adc_control"),
                data_pads    = platform.request("adc_data"),
                sys_clk_freq = sys_clk_freq,
                frame_polarity=frame_polarity[variant],
                data_polarity=adc_polarity[variant]
            )

            # ADC -> PCIe.
            self.sync += self.adc.source.connect(self.pcie_dma0.sink)

        # Analyzer -----------------------------------------------------------------------------

        if with_analyzer:
            analyzer_signals = [
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 1024,
                clock_domain = "adc_frame",
                samplerate   = sys_clk_freq,
                csr_csv      = "test/analyzer.csv"
            )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=Platform, description="LitePCIe SoC on ThunderScope")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--variant",   default="dev",     help="Board variant [prod, dev, a200t, a100t, a50t].")
    target_group.add_argument("--flash",     action="store_true", help="Flash bitstream.")
    target_group.add_argument("--driver",    action="store_true", help="Generate PCIe driver.")
    target_group.add_argument("--cable",     default="digilent_hs2", help="JTAG cable name.")
    target_group.add_argument("--driver-dir", default="software", help="Directory to store driver")

    
    args = parser.parse_args()

    # Build SoC.
    soc = BaseSoC(variant = args.variant,  **parser.soc_argdict)

    builder  = Builder(soc,  **parser.builder_argdict)
    os.makedirs(builder.gateware_dir, exist_ok=True)
    shutil.copyfile(f"bin/barrierA.bin", f"{builder.gateware_dir}/barrierA.bin")
    shutil.copyfile(f"bin/barrierB.bin", f"{builder.gateware_dir}/barrierB.bin")
    builder.build(run=args.build)

    # Generate LitePCIe Driver.
    if args.driver:
        generate_litepcie_software(soc, args.driver_dir)

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(variant = args.variant, cable = args.cable)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    # Flash Bitstream.
    if args.flash:
        prog = soc.platform.create_programmer(variant = args.variant, cable = args.cable)
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext="_full.bin"))

if __name__ == "__main__":
    main()
