
from litex.build.generic_platform import *

from litex.build.xilinx import Xilinx7SeriesPlatform
from litex.build.openfpgaloader import OpenFPGALoader

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

class ThunderscopePlatform(Xilinx7SeriesPlatform):
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
                                toolchain=toolchain,
                                name = "thunderscope")

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
        
