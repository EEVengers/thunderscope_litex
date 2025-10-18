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

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import *
from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone

from litex.soc.cores.clock import *
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

from thunderscope_platform import *


def get_commit_hash_string():
    # Get the hash for the current commit
    commit_hash = subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode('ascii')
    # Check if the repo is dirty
    repo_dirty = bool(subprocess.check_output(['git', 'status', '--porcelain']).decode('ascii').strip())
    return commit_hash, repo_dirty


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
        platform = ThunderscopePlatform(variant=variant)

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
                # Gateware Version Info
                self._gw_rev = CSRStatus(name="gw_rev", size=32, reset=0x0000_0300)

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
                clock_domain = "sys",
                samplerate   = sys_clk_freq,
                csr_csv      = "test/analyzer.csv"
            )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=ThunderscopePlatform, description="LitePCIe SoC on ThunderScope")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--variant",   default="dev",     help="Board variant [prod, dev, a200t, a100t, a50t].")
    target_group.add_argument("--flash",     action="store_true", help="Flash bitstream.")
    target_group.add_argument("--driver",    action="store_true", help="Generate PCIe driver.")
    target_group.add_argument("--cable",     default="digilent_hs2", help="JTAG cable name.")
    target_group.add_argument("--driver-dir", default="build/software", help="Directory to store driver")
    target_group.add_argument("--with_jtagbone", default=True, help="Enable JTAG Master.")

    
    args = parser.parse_args()

    # Build SoC.
    soc = BaseSoC(variant = args.variant, **parser.soc_argdict)

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
