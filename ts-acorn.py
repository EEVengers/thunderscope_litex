#!/usr/bin/env python3
import os

from migen import *

from litex.gen import *

from litex_boards.platforms import sqrl_acorn
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer
from litex.build.openocd import OpenOCD
from litex.build.generic_platform import *

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
# GPIO
_spi_io = [   
    # SPI.
    ("main_spi", 0,
        Subsignal("clk",  Pins("J2")),
        Subsignal("mosi", Pins("J5"), Misc("PULLUP True")),
        Subsignal("cs_n", Pins("H5"), Misc("PULLUP True")),
        # Subsignal("miso", Pins("K2"), Misc("PULLUP True")),
        Misc("SLEW=FAST"),
        IOStandard("LVCMOS33")
    )
]
_i2c_io = [   
    # I2C.
    ("test_i2c", 0,
        Subsignal("scl", Pins("Y9")),
        Subsignal("sda", Pins("Y7")),
        IOStandard("LVCMOS33")
    )
]

_fe_comp_io = [
    # Probe Compensation.
    ("fe_probe_compensation", 0, Pins("K2"), IOStandard("LVCMOS33")),

]

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst          = Signal()
        self.cd_sys       = ClockDomain()
        self.cd_idelay    = ClockDomain()

        # Clk/Rst
        clk200 = platform.request("clk200")

        # PLL.
        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, reset_buf="bufg")
        pll.create_clkout(self.cd_idelay, 200e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)
        platform.add_period_constraint(self.cd_idelay.clk, 1e9/200e6)

        # self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, variant="cle-215+", sys_clk_freq=int(150e6),
        with_led_chaser = True,
        with_pcie     = True,
        with_frontend = True,
        with_adc      = True,
        with_jtagbone = True,
        with_analyzer = False,
        **kwargs):
        platform = sqrl_acorn.Platform(variant=variant)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident="LiteX Thunderscope on Acorn CLE-101/215(+)",
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
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.comb += platform.request("pcie_clkreq_n").eq(0)
            self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
                data_width = 128,
                bar0_size  = 0x20000
            )
            self.add_pcie(phy=self.pcie_phy, ndmas=1, dma_buffering_depth=1024*16, max_pending_requests=4, address_width=64)

            # ICAP (For FPGA reload over PCIe).
            from litex.soc.cores.icap import ICAP
            self.submodules.icap = ICAP()
            self.icap.add_reload()
            self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

            # # Flash (For SPIFlash update over PCIe).
            # from litex.soc.cores.gpio import GPIOOut
            # from litex.soc.cores.spi_flash import S7SPIFlash
            # self.flash_cs_n = GPIOOut(platform.request("flash_cs_n"))
            # self.flash      = S7SPIFlash(platform.request("flash"), sys_clk_freq, 25e6)

        
        # I2C Bus:
        # - Trim DAC (MCP4728 @ 0x61).
        # - PLL      (LMK61E2 @ 0x58).
        platform.add_extension(_i2c_io)
        self.submodules.i2c = I2CMaster(platform.request("test_i2c"))        

        # Probe Compensation.
        platform.add_extension(_fe_comp_io)
        self.submodules.probe_compensation = PWM(
            pwm = platform.request("fe_probe_compensation"),
            default_enable = 1,
            default_width  = int(1e-3*sys_clk_freq/2),
            default_period = int(1e-3*sys_clk_freq)
        )

        platform.add_extension(_spi_io)
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

                    if control_pads is not None:
                        # Power.
                        self.comb += control_pads.fe_en.eq(self._control.fields.fe_en)

                        # Coupling.
                        self.comb += control_pads.coupling.eq(self._control.fields.coupling)

                        # Attenuation.
                        self.comb += control_pads.attenuation.eq(self._control.fields.attenuation)


            self.submodules.frontend = Frontend(
                control_pads     = None,
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
                    if control_pads is not None:
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
                    
            self.submodules.adc = ADC(
                control_pads = None,
                data_pads    = None,
                sys_clk_freq = sys_clk_freq,
            )

            # ADC -> PCIe.
            self.comb += self.adc.source.connect(self.pcie_dma0.sink)

            # Analyzer -----------------------------------------------------------------------------

            if with_analyzer:
                analyzer_signals = [
                    self.adc.source
                ]
                self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                    depth        = 1024,
                    clock_domain = "sys",
                    samplerate   = sys_clk_freq,
                    csr_csv      = "test/analyzer.csv"
                )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sqrl_acorn.Platform, description="LiteX SoC on Acorn CLE-101/215(+).")
    parser.add_target_argument("--flash",           action="store_true",       help="Flash bitstream.")
    parser.add_target_argument("--variant",         default="cle-215+",        help="Board variant (cle-215+, cle-215 or cle-101).")
    pcieopts = parser.target_group.add_mutually_exclusive_group()
    parser.add_target_argument("--driver",          action="store_true", help="Generate PCIe driver.")
    args = parser.parse_args()

    soc = BaseSoC(
        variant         = args.variant,
        with_led_chaser =True,
        with_pcie       = True,
        with_frontend   = True,
        with_adc        = True,
        **parser.soc_argdict
    )

    builder  = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = OpenOCD("scripts/tigard.cfg", "bscan_spi_xc7a100t.bit")
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = OpenOCD("scripts/tigard.cfg", "bscan_spi_xc7a100t.bit")
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
