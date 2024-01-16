#!/usr/bin/env python3
import os

from migen import *

from litex.gen import *
from litex.build.generic_platform import *
from litex_boards.platforms import sqrl_acorn

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA
from litex.soc.cores.bitbang import I2CMaster

from litedram.modules import MT41K512M16
from litedram.phy import s7ddrphy

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

# from litex_boards.targets.sqrl_acorn import BaseSoC
from litex.build.openocd import OpenOCD


# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst          = Signal()
        self.cd_sys       = ClockDomain()
        self.cd_sys4x     = ClockDomain()
        self.cd_sys4x_dqs = ClockDomain()
        self.cd_idelay    = ClockDomain()

        # Clk/Rst
        clk200 = platform.request("clk200")

        # PLL
        self.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_idelay,    200e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, variant="cle-215+", sys_clk_freq=100e6,
        with_led_chaser = True,
        with_pcie       = False,
        with_sata       = False,
        **kwargs):
        platform = sqrl_acorn.Platform(variant=variant)

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Acorn CLE-101/215(+)", **kwargs)

        # XADC -------------------------------------------------------------------------------------
        self.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        # if not self.integrated_main_ram_size:
        #     self.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
        #         memtype          = "DDR3",
        #         nphases          = 4,
        #         sys_clk_freq     = sys_clk_freq,
        #         iodelay_clk_freq = 200e6)
        #     self.add_sdram("sdram",
        #         phy           = self.ddrphy,
        #         module        = MT41K512M16(sys_clk_freq, "1:4"),
        #         l2_cache_size = kwargs.get("l2_size", 8192)
        #     )

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.comb += platform.request("pcie_clkreq_n").eq(0)
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
                data_width = 128,
                bar0_size  = 0x20000)
            self.add_pcie(phy=self.pcie_phy, ndmas=1, max_pending_requests=8,
                          dma_buffering_depth=4096, address_width=64)
            # self.add_pcie(phy=self.pcie_phy, ndmas=1, address_width=64)
            platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)

            # ICAP (For FPGA reload over PCIe).
            from litex.soc.cores.icap import ICAP
            self.icap = ICAP()
            self.icap.add_reload()
            self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

            # Flash (For SPIFlash update over PCIe).
            from litex.soc.cores.gpio import GPIOOut
            from litex.soc.cores.spi_flash import S7SPIFlash
            self.flash_cs_n = GPIOOut(platform.request("flash_cs_n"))
            self.flash      = S7SPIFlash(platform.request("flash"), sys_clk_freq, 25e6)


            platform.add_extension([
                ("fe_control", 0,
                        Subsignal("fe_en",      Pins("J21"), IOStandard("LVCMOS33")), # TPS7A9101/LDO & LM27761 Enable.
                        Subsignal("coupling",    Pins("H20 K19 H19 N18"), IOStandard("LVCMOS33")),
                        Subsignal("attenuation", Pins("G20 K18 J19 N19"), IOStandard("LVCMOS33")),
                        IOStandard("LVCMOS33"),
                    ),
                    # SPI
                    ("spi", 0,
                        Subsignal("clk",  Pins("K21")),
                        Subsignal("cs_n", Pins("K13 J22 L20 M21 L18")),
                        Subsignal("mosi", Pins("K22")),
                        IOStandard("LVCMOS33"),
                    ),
                    ("i2c", 0,
                            Subsignal("sda", Pins("J14")),
                            Subsignal("scl", Pins("H14")),
                            IOStandard("LVCMOS33"),
                        ),
                        # Control / Status / SPI.
                    ("adc_control", 0,
                        Subsignal("acq_en", Pins("J20")), # TPS7A9101/LDO Enable.
                        Subsignal("osc_oe", Pins("K14")), # LMK61E2/PLL Output Enable.
                        IOStandard("LVCMOS33"),
                    )
            ])
             # Control / Status.
    

        # I2C Bus:
        # - Trim DAC (MCP4728 @ 0x61).
        # - PLL      (LMK61E2 @ 0x58).
        self.i2c = I2CMaster(platform.request("i2c"))

        

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sqrl_acorn.Platform, description="LiteX SoC on Acorn CLE-101/215(+).")
    parser.add_target_argument("--flash",           action="store_true",       help="Flash bitstream.")
    parser.add_target_argument("--variant",         default="cle-215+",        help="Board variant (cle-215+, cle-215 or cle-101).")
    parser.add_target_argument("--sys-clk-freq",    default=100e6, type=float, help="System clock frequency.")
    pcieopts = parser.target_group.add_mutually_exclusive_group()
    pcieopts.add_argument("--with-pcie",            action="store_true", help="Enable PCIe support.")
    parser.add_target_argument("--driver",          action="store_true", help="Generate PCIe driver.")
    parser.add_target_argument("--with-spi-sdcard", action="store_true", help="Enable SPI-mode SDCard support (requires SDCard adapter on P2).")
    pcieopts.add_argument("--with-sata",            action="store_true", help="Enable SATA support (over PCIe2SATA).")
    args = parser.parse_args()

    soc = BaseSoC(
        variant      = args.variant,
        sys_clk_freq = args.sys_clk_freq,
        with_pcie    = args.with_pcie,
        with_sata    = args.with_sata,
        with_led_chaser=True,
        **parser.soc_argdict
    )
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()

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
