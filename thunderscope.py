#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer
from litex.build.openocd import OpenOCD

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.gpio import GPIOOut
from litex.soc.cores.spi import SPIMaster
from litex.soc.cores.pwm import PWM

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

from litescope import LiteScopeAnalyzer

from peripherals.had1511_adc import HAD1511ADC
from peripherals.trigger import Trigger

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
    ("user_led_n", 0, Pins("T21"), IOStandard("LVCMOS33")), # Red.

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
        Subsignal("coupling",    Pins("H20 K19 H19 N18"), IOStandard("LVCMOS33")),
        Subsignal("attenuation", Pins("G20 K18 J19 N19"), IOStandard("LVCMOS33")),
        Subsignal("term",        Pins("H22 L19 L21 M18"), IOStandard("LVCMOS33"))
    ),

    # SPI
    ("main_spi", 0,
        Subsignal("clk",  Pins("K21")),
        Subsignal("cs_n", Pins("J22 L20 M21 L18 K13")),
        Subsignal("mosi", Pins("K22")),
        IOStandard("LVCMOS33"),
    ),

    # I2C bus.
    # --------
    # - Trim DAC (MCP4728 @ 0xC0).
    # - PLL      (LMK61E2 @ 0x58).
    # - ClockGen (ZL30250 @ 0xD8).
    # - TODO: Digi-pot @ 0x58.

    ("i2c", 0,
        Subsignal("sda", Pins("J14")),
        Subsignal("scl", Pins("H14")),
        IOStandard("LVCMOS33"),
    ),

    # ADC / HMCAD1511.
    # ----------------

    # Control / Status / SPI.
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
        IOStandard("LVDS_25"),
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
    ("user_led_n", 0, Pins("U17"), IOStandard("LVCMOS33")), # Red.

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
        Subsignal("coupling",    Pins("N6 M2 T2 M6"), IOStandard("LVCMOS33")),
        Subsignal("attenuation", Pins("M1 M5 P1 L4"), IOStandard("LVCMOS33")),
        Subsignal("term",        Pins("P3 N2 P5 J4"), IOStandard("LVCMOS33"))
    ),

    # SPI
    ("main_spi", 0,
        Subsignal("clk",  Pins("K2")),
        Subsignal("cs_n", Pins("N1 R3 K1 R6 J5")),
        Subsignal("mosi", Pins("L3")),
        IOStandard("LVCMOS33"),
    ),

    # I2C bus.
    # --------
    # - Trim DAC (MCP4728 @ 0xC0).
    # - PLL      (LMK61E2 @ 0x58).
    # - ClockGen (ZL30250 @ 0xD8).
    # - TODO: Digi-pot @ 0x58.

    ("i2c", 0,
        Subsignal("sda", Pins("N4")),
        Subsignal("scl", Pins("K5")),
        IOStandard("LVCMOS33"),
    ),

    # ADC / HMCAD1511.
    # ----------------

    # Control / Status / SPI.
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
        IOStandard("LVDS_25"),
        Misc("DIFF_TERM=FALSE"),
    ),

    # SYNC
    # ----------------
    ("sync", 0, Pins("P6"), IOStandard("LVCMOS33"))
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxPlatform):
    device = {
        "a100t" : {"fpga": "xc7a100tfgg484-2", "io": a7_484_io, "flash": "bscan_spi_xc7a100t.bit"},
        "a200t" : {"fpga": "xc7a200tfbg484-2", "io": a7_484_io, "flash": "bscan_spi_xc7a200t.bit"},
        "a50t"  : {"fpga": "xc7a50tcsg325-2",  "io": a7_325_io, "flash": "bscan_spi_xc7a50t.bit"},
    }
    def __init__(self, toolchain="vivado", variant="a100t"):

        XilinxPlatform.__init__(self, 
                                self.device[variant]["fpga"],
                                self.device[variant]["io"],
                                toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 16 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]

        self.toolchain.additional_commands = [
            # Non-Multiboot SPI-Flash bitstream generation.
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin",
            # Multiboot bitstreams
            "write_bitstream -force -bin_file {build_name}.bit",
            "set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR 0x0097FC00 [current_design]",
            "write_bitstream -force -bin_file {build_name}_gold.bit",
            "write_cfgmem -force -format mcs -size 32 -interface SPIx4 -loadbit \"up 0x00000000 {build_name}_gold.bit up 0x00980000 {build_name}.bit\" -loaddata \"up 0x0097FC00 ../../../cfg/timer1.bin up 0x01300000 ../../../cfg/timer2.bin\" {build_name}_full.mcs",
            "write_cfgmem -force -format mcs -size 32 -interface SPIx4 -loadbit \"up 0x00980000 {build_name}.bit\" {build_name}_update.mcs"
        ]

    def create_programmer(self, name='openocd', variant="a100t"):
        if name == 'openocd':
            return OpenOCD("openocd_xc7_ft232.cfg", self.device[variant]["flash"])
        elif name == 'vivado':
            # TODO: some board versions may have s25fl128s
            return VivadoProgrammer(flash_part='s25fl256sxxxxxx0-spi-x1_x2_x4')

    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("adc_data:lclk_p", loose=True), 2e9/1000e6)

# CRG ----------------------------------------------------------------------------------------------


class CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_idelay = ClockDomain()

        # CFGM Clk ~65MHz.
        cfgm_clk      = Signal()
        cfgm_clk_freq = int(65e6)
        self.specials += Instance("STARTUPE2",
            i_CLK       = 0,
            i_GSR       = 0,
            i_GTS       = 0,
            i_KEYCLEARB = 1,
            i_PACK      = 0,
            i_USRCCLKO  = cfgm_clk,
            i_USRCCLKTS = 0,
            i_USRDONEO  = 1,
            i_USRDONETS = 1,
            o_CFGMCLK   = cfgm_clk
        )
        platform.add_period_constraint(cfgm_clk, 1e9/65e6)

        # PLL.
        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        # Trenz modules have 50MHz Clock, TS Module has 25MHz Clock
        clk = platform.request("clk50", loose = True)
        if clk is not None:
            pll.register_clkin(clk, 50e6)
        else:
            pll.register_clkin(platform.request("clk25"), 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, reset_buf="bufg")
        pll.create_clkout(self.cd_idelay, 200e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)
        platform.add_period_constraint(self.cd_idelay.clk, 1e9/200e6)

        # IDELAYCTRL.
        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)
        # self.rst          = Signal()
        # self.cd_sys       = ClockDomain()
        # self.cd_sys4x     = ClockDomain()
        # self.cd_sys4x_dqs = ClockDomain()
        # self.cd_idelay    = ClockDomain()

        # # # #

        # self.pll = pll = S7PLL(speedgrade=-2)
        # self.comb += pll.reset.eq(self.rst)
        # pll.register_clkin(platform.request("clk50"), 50e6)
        # pll.create_clkout(self.cd_sys,       sys_clk_freq)
        # pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        # pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        # pll.create_clkout(self.cd_idelay,    200e6)
        # platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        # self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, sys_clk_freq=int(125e6),
        variant       ="a100t",
        with_pcie     = True,
        with_frontend = True,
        with_adc      = True,
        with_jtagbone = True,
        with_analyzer = False,
    ):
        platform = Platform(variant=variant)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LitePCIe SoC on ThunderScope",
            ident_version = True,
        )

        # JTAGBone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()

        # XADC -------------------------------------------------------------------------------------
        self.submodules.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.submodules.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request("user_led_n"),
            sys_clk_freq = sys_clk_freq,
            polarity     = 1,
        )
        self.leds.add_pwm(default_width=128, default_period=1024) # Default to 1/8 to reduce brightness.

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
                data_width = 128,
                bar0_size  = 0x20000
            )
            self.add_pcie(phy=self.pcie_phy, ndmas=1, dma_buffering_depth=8192, max_pending_requests=4)
            # FIXME: Apply it to all targets (integrate it in LitePCIe?).
            #platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks userclk2] -asynchronous", sys_clk=self.crg.cd_sys.clk)
            #platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks clk_125mhz] -asynchronous", sys_clk=self.crg.cd_sys.clk)
            #platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks clk_250mhz] -asynchronous", sys_clk=self.crg.cd_sys.clk)
            #platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks clk_125mhz] -group [get_clocks clk_250mhz] -asynchronous")

            # ICAP (For FPGA reload over PCIe).
            from litex.soc.cores.icap import ICAP
            self.submodules.icap = ICAP()
            self.icap.add_reload()
            self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # Frontend / ADC ---------------------------------------------------------------------------

        # I2C Bus:
        # - Trim DAC (MCP4728 @ 0x61).
        # - PLL      (LMK61E2 @ 0x58).
        self.i2c = I2CMaster(platform.request("i2c"))

        # Probe Compensation.
        self.submodules.probe_compensation = PWM(
            pwm = platform.request("fe_probe_compensation"),
            default_enable = 1,
            default_width  = int(1e-3*sys_clk_freq/2),
            default_period = int(1e-3*sys_clk_freq)
        )

        
        main_spi_pads = platform.request("main_spi")
        main_spi_clk_freq = 1e6
        main_spi_pads.miso = Signal()
        self.submodules.main_spi = main_spi = SPIMaster(
            pads         = main_spi_pads,
            data_width   = 24,
            sys_clk_freq = sys_clk_freq,
            spi_clk_freq = main_spi_clk_freq
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
                            ("``0b0``", " 1X-Attenuation (one bit per channel)."),
                            ("``0b1``", "10X-Attenuation (one bit per channel)."),
                        ]),
                    ])
                    # # #

                    # Power.
                    self.comb += control_pads.fe_en.eq(self._control.fields.fe_en)

                    # Coupling.
                    self.comb += control_pads.coupling.eq(self._control.fields.coupling)

                    # Attenuation.
                    self.comb += control_pads.attenuation.eq(self._control.fields.attenuation)


            self.submodules.frontend = Frontend(
                control_pads     = platform.request("fe_control"),
                sys_clk_freq     = sys_clk_freq,
            )

        # ADC.
        if with_adc:

            class ADC(Module, AutoCSR):
                def __init__(self, control_pads, data_pads, sys_clk_freq,
                    data_width   = 128, data_polarity = [1, 1, 0, 1, 1, 1, 1, 1]
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
                   

                    # Data Source.
                    self.source = stream.Endpoint([("data", data_width)])

                    # # #

                    # Control-Path -----------------------------------------------------------------

                    # Control.
                    self.comb += [
                        control_pads.acq_en.eq(self._control.fields.acq_en),
                        control_pads.osc_oe.eq(self._control.fields.osc_en),
                    ]

                    # Data-Path --------------------------------------------------------------------

                    # Trigger.
                    self.submodules.trigger = Trigger()

                    # HAD1511.
                    self.submodules.had1511 = HAD1511ADC(data_pads, sys_clk_freq, lanes_polarity=data_polarity)

                    # Gate/Data-Width Converter.
                    self.submodules.gate = stream.Gate([("data", 64)], sink_ready_when_disabled=True)
                    self.submodules.conv = stream.Converter(64, data_width)
                    self.comb += self.gate.enable.eq(self.trigger.enable)

                    # Pipeline.
                    self.submodules += stream.Pipeline(
                        self.had1511,
                        self.gate,
                        self.conv,
                        self.source
                    )

            adc_polarity = {"a100t" : [1, 1, 0, 1, 1, 1, 1, 1],
                            "a200t" : [1, 1, 0, 1, 1, 1, 1, 1],
                            "a50t"  : [0, 0, 1, 1, 0, 1, 1, 1]}

            self.submodules.adc = ADC(
                control_pads = platform.request("adc_control"),
                data_pads    = platform.request("adc_data"),
                sys_clk_freq = sys_clk_freq,
                data_polarity=adc_polarity[variant]
            )

            # ADC -> PCIe.
            self.comb += self.adc.source.connect(self.pcie_dma0.sink)

            # ADC Debug
            clk_count = Signal(32)
            clk_div = platform.request("sync")
            self.sync.adc_test += clk_count.eq(clk_count + 1)
            # self.sync.adc += clk_count.eq(clk_count + 1)
            self.comb += clk_div.eq(clk_count[8])

            # Analyzer -----------------------------------------------------------------------------

            if with_analyzer:
                analyzer_signals = [
                    self.adc.source,
                    self.adc.had1511.bitslip,
                    self.adc.had1511.fclk,
                ]
                self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                    depth        = 1024,
                    clock_domain = "sys",
                    samplerate   = sys_clk_freq,
                    csr_csv      = "test/analyzer.csv"
                )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LitePCIe SoC on ThunderScope")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--variant",   default="a100t",     help="Board variant (a200t, a100t or a50t).")
    target_group.add_argument("--build",     action="store_true", help="Build bitstream.")
    target_group.add_argument("--load",      action="store_true", help="Load bitstream.")
    target_group.add_argument("--flash",     action="store_true", help="Flash bitstream.")
    target_group.add_argument("--driver",    action="store_true", help="Generate PCIe driver.")
    args = parser.parse_args()

    # Build SoC.
    soc = BaseSoC(variant = args.variant)

    builder  = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    # Generate LitePCIe Driver.
    if args.driver:
        generate_litepcie_software(soc, "software")

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    # Flash Bitstream.
    if args.flash:
        prog = soc.platform.create_programmer(name="vivado", variant = args.variant)
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
