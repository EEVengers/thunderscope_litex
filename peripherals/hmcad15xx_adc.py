#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2020-2021 Felix Domke <tmbinc@elitedvb.net>
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import time

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from migen.genlib.misc import WaitTimer

from litex.gen import *
from litex.build.io import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from peripherals.spi import *
from peripherals.downsampling import DownSampling
from peripherals.byteShuffler import ByteShuffler

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# Core.

HAD1511_CORE_CONTROL_FRAME_RST = (1 << 0)
HAD1511_CORE_CONTROL_DELAY_RST = (1 << 1)
HAD1511_CORE_CONTROL_DELAY_INC = (1 << 2)
HAD1511_CORE_CONTROL_STAT_RST  = (1 << 3)

HAD1511_CORE_RANGE_STAT_MIN    = (1 << 0)
HAD1511_CORE_RANGE_STAT_MAX    = (1 << 8)

# ADC.

HAD1511_ADC_GAIN_0DB = 0
HAD1511_ADC_GAIN_2DB = 2
HAD1511_ADC_GAIN_4DB = 4
HAD1511_ADC_GAIN_6DB = 6
HAD1511_ADC_GAIN_9DB = 9

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# Layouts ------------------------------------------------------------------------------------------

had1511_phy_layout = ["fclk_p", "fclk_n", "lclk_p", "lclk_n", "d_p", "d_n"]

# SERDES Lane

"""
                                                                                                Bitclock  :  Sys Clock                
                                                                                                          :             
                        +------------+      +------------+      +----------+     +-----------+       +----------+              
                        |            |      |            |      |          |     |           |       |    :     |               
 Differential Pads ---->| IBUFDS    >|-- -->|  IDELAY   >|----->|  IDDR   >|-/2->|  Packer  >|--/16->|  AFIFO   |---/16----> d
                        |            |      |            |      |          |     |           |       |    :     |                
                        +------------+      +------------+      +----------+     +-----------+       +----------+                      
                                                                                       ^ ^             ^  :                            
                                                                                       | |             |  :                                 
             Frame --------------------------------------------------------------------+ |             |  :
        Frame Edge ----------------------------------------------------------------------+-------------+  :
"""

class LVDSLane(LiteXModule):
    def __init__(self, d_p, d_n, frame_edge, polarity, delay_inc, delay_rst):
        d_no_delay = Signal()
        d_delayed  = Signal()
        d_sdr      = Signal(2)
        d_shift    = Signal(16)
        self.d = d = Signal(16)


        self.specials += [
            Instance("IBUFDS",
                i_I  = d_p,
                i_IB = d_n,
                o_O  = d_no_delay
            ),
            Instance("IDELAYE2",
                p_DELAY_SRC             = "IDATAIN",
                p_SIGNAL_PATTERN        = "DATA",
                p_CINVCTRL_SEL          = "FALSE",
                p_HIGH_PERFORMANCE_MODE = "TRUE",
                p_REFCLK_FREQUENCY      = 200.0,
                p_PIPE_SEL              = "FALSE",
                p_IDELAY_TYPE           = "VARIABLE",
                p_IDELAY_VALUE          = 0,

                i_C        = ClockSignal("sys"),
                i_LD       = delay_rst,
                i_CE       = delay_inc,
                i_LDPIPEEN = 0,
                i_INC      = 1,

                i_IDATAIN  = d_no_delay,
                o_DATAOUT  = d_delayed
            ),
            Instance("IDDR",
                i_C  = ClockSignal("adc"), # Clock
                i_CE = 1,
                i_R  = ResetSignal("adc_reset"), # Reset
                i_S  = 0,
                i_D  = d_delayed,
                **{f"o_Q{n+1}": d_sdr[n] for n in range(2)},
            )
        ]

       
        # Variable Width Deserializer

        self.sync.adc += [
            If( frame_edge, d.eq(d_shift), d_shift[2:16].eq(0)).Else(
            d_shift[2:16].eq(d_shift[0:14])),
            d_shift[0:1].eq(d_sdr if polarity == 0 else (d_sdr ^ 0x3))
        ]
        
        # Async Fifo to put data in sys clk domain
        fifo = stream.AsyncFIFO([("data", 16)])
        fifo = ClockDomainsRenamer({"write": "adc_frame", "read": "sys"})(fifo)
        self.submodules += fifo

        self.comb += [
            fifo.sink.valid.eq(frame_edge),
            fifo.sink.data.eq(d)
        ]


# HAD1520 ADC --------------------------------------------------------------------------------------

class HMCAD15XXADC(LiteXModule):
    def __init__(self, pads, sys_clk_freq, frame_polarity, lanes_polarity=[0]*8, clock_domain="sys"):
        # Parameters.
        if pads is not None:
            nchannels = len(pads.d_p)
            for name in had1511_phy_layout:
                assert hasattr(pads, name)
        else:
            nchannels = 8

        # ADC stream.
        self.source = source = stream.Endpoint([("data", nchannels*16)])

        # Control/Status.
        self._control      = CSRStorage(fields=[
            CSRField("frame_rst", offset=0, size=1, pulse=True, description="Frame clock reset."),
            CSRField("delay_rst", offset=1, size=1, pulse=True, description="Sampling delay reset."),
            CSRField("delay_inc", offset=2, size=1, pulse=True, description="Sampling delay increment."),
            CSRField("stat_rst",  offset=3, size=1, pulse=True, description="Statistics reset.")
        ])
        self._status       = CSRStatus() # Unused (for now).
        self._downsampling = CSRStorage(32, description="ADC Downsampling ratio.")
        self._range        = CSRStatus(fields=[
            CSRField("min01", size=16, offset= 0, description="ADC0/1 Min value since last stat_rst."),
            CSRField("max01", size=16, offset=16, description="ADC0/1 Max value since last stat_rst."),
            CSRField("min23", size=16, offset=32, description="ADC2/3 Min value since last stat_rst."),
            CSRField("max23", size=16, offset=48, description="ADC2/3 Max value since last stat_rst."),
        ])
        self._bitslip_count = CSRStatus(32, description="ADC bitslip count (with rollover).")
        self._sample_count  = CSRStatus(32, description="ADC samples count since last stat_rst.")
        self._data_channels = CSRStorage(fields=[
            CSRField("shuffle",     offset=0, size=3, reset=0, description="Number of enabled channels to control shuffling of samples",
                    values=[
                        (0, "Passthrough."),
                        (1, "2-Channel Shuffle."),
                        (2, "4-Channel Shuffle."),
                        (3, "2-Channel 16bit Shuffle."),
                        (4, "4-Channel 16bit Shuffle.")
                    ]),
            CSRField("run_length",  offset=3, size=5, reset=1, description="Control Run-Length of samples for each channel ordered next to each other"),
            CSRField("data_width",offset=8, size=1, pulse=False, description="Select the deserializer data width.",
                    values=[
                        (0b0, "8-bit"),
                        (0b1, "12-bit")
                    ])
        ])

        # # #


        # Bit and Frame clocking.
        # ---------

        if pads is not None:
            self.clock_domains.cd_adc       = ClockDomain() # ADC Bitclock.

            adc_clk = Signal()
            adc_frame = Signal()
            adc_frame_delayed = Signal()
            self.specials += Instance("IBUFDS",
                i_I  = pads.lclk_p,
                i_IB = pads.lclk_n,
                o_O  = adc_clk
            )
            self.specials += Instance("BUFIO",
                i_I = adc_clk,
                o_O = ClockSignal("adc")
            )
            
            # Receive & Deserialize Frame clock to use it as a delimiter for the data.
            self.specials += [
                Instance("IBUFDS",
                    i_I  = pads.fclk_p,
                    i_IB = pads.fclk_n,
                    o_O  = adc_frame
                ),
                Instance("IDELAYE2",
                    p_DELAY_SRC             = "IDATAIN",
                    p_SIGNAL_PATTERN        = "DATA",
                    p_CINVCTRL_SEL          = "FALSE",
                    p_HIGH_PERFORMANCE_MODE = "TRUE",
                    p_REFCLK_FREQUENCY      = 200.0,
                    p_PIPE_SEL              = "FALSE",
                    p_IDELAY_TYPE           = "VARIABLE",
                    p_IDELAY_VALUE          = 0,

                    i_C        = ClockSignal("sys"),
                    i_LD       = self._control.fields.delay_rst,
                    i_CE       = self._control.fields.delay_inc,
                    i_LDPIPEEN = 0,
                    i_INC      = 1,

                    i_IDATAIN  = adc_frame,
                    o_DATAOUT  = adc_frame_delayed
                )
            ]
            self.specials += [AsyncResetSynchronizer(self.cd_adc,  self._control.fields.frame_rst)]

        # LVDS Reception & Deserialization.
        # ---------------------------------

        if pads is not None:
            # self.bitslip       = bitslip       = Signal()
            # self.fclk          = fclk          = Signal(16)
            # self.fclk_no_delay = fclk_no_delay = Signal()
            # self.fclk_delayed  = fclk_delayed  = Signal()

            # # Receive & Deserialize Frame clock to use it as a delimiter for the data.
            # self.specials += [
            #     Instance("IBUFDS",
            #         i_I  = pads.fclk_p,
            #         i_IB = pads.fclk_n,
            #         o_O  = fclk_no_delay
            #     ),
            #     Instance("IDELAYE2",
            #         p_DELAY_SRC             = "IDATAIN",
            #         p_SIGNAL_PATTERN        = "DATA",
            #         p_CINVCTRL_SEL          = "FALSE",
            #         p_HIGH_PERFORMANCE_MODE = "TRUE",
            #         p_REFCLK_FREQUENCY      = 200.0,
            #         p_PIPE_SEL              = "FALSE",
            #         p_IDELAY_TYPE           = "VARIABLE",
            #         p_IDELAY_VALUE          = 0,

            #         i_C        = ClockSignal("sys"),
            #         i_LD       = self._control.fields.delay_rst,
            #         i_CE       = self._control.fields.delay_inc,
            #         i_LDPIPEEN = 0,
            #         i_INC      = 1,

            #         i_IDATAIN  = fclk_no_delay,
            #         o_DATAOUT  = fclk_delayed
            #     ),
                # Instance("ISERDESE2",
                #     p_DATA_WIDTH     = 8,
                #     p_DATA_RATE      = "DDR",
                #     p_SERDES_MODE    = "MASTER",
                #     p_INTERFACE_TYPE = "NETWORKING",
                #     p_NUM_CE         = 1,
                #     p_IOBDELAY       = "IFD",
                #     i_DDLY    = fclk_delayed,
                #     i_CE1     = 1,
                #     i_RST     =  ResetSignal("adc8_frame"),
                #     i_CLK     =  ClockSignal("adc"),
                #     i_CLKB    = ~ClockSignal("adc"),
                #     i_CLKDIV  =  ClockSignal("adc8_frame"),
                #     i_BITSLIP = bitslip,
                #      **{f"o_Q{n+1}": fclk[8-1-n] for n in range(8)},
                # )
            # ]


            # Receive & Deserialize Data.
            self.adc_source = adc_source = stream.Endpoint([("data", nchannels*16)])
            self.comb += adc_source.valid.eq(adc_frame_delayed)
            channels = {}
            for i in range(nchannels):
                channels[i] = LVDSLane(pads.d_p[i], pads.d_n[i],
                                       adc_frame_delayed, lanes_polarity[i],
                                       self._control.fields.delay_inc, self._control.fields.delay_rst)
                self.comb += adc_source.data[16*i:16*(i+1)].eq(channels[i].d)


                # d_no_delay = Signal()
                # d_delayed  = Signal()
                # d          = Signal(8)
                # self.specials += [
                #     Instance("IBUFDS",
                #         i_I  = pads.d_p[i],
                #         i_IB = pads.d_n[i],
                #         o_O  = d_no_delay
                #     ),
                #     Instance("IDELAYE2",
                #         p_DELAY_SRC             = "IDATAIN",
                #         p_SIGNAL_PATTERN        = "DATA",
                #         p_CINVCTRL_SEL          = "FALSE",
                #         p_HIGH_PERFORMANCE_MODE = "TRUE",
                #         p_REFCLK_FREQUENCY      = 200.0,
                #         p_PIPE_SEL              = "FALSE",
                #         p_IDELAY_TYPE           = "VARIABLE",
                #         p_IDELAY_VALUE          = 0,

                #         i_C        = ClockSignal("sys"),
                #         i_LD       = self._control.fields.delay_rst,
                #         i_CE       = self._control.fields.delay_inc,
                #         i_LDPIPEEN = 0,
                #         i_INC      = 1,

                #         i_IDATAIN  = d_no_delay,
                #         o_DATAOUT  = d_delayed
                #     ),
                #     Instance("ISERDESE2",
                #         p_DATA_WIDTH     = 8,
                #         p_DATA_RATE      = "DDR",
                #         p_SERDES_MODE    = "MASTER",
                #         p_INTERFACE_TYPE = "NETWORKING",
                #         p_NUM_CE         = 1,
                #         p_IOBDELAY       = "IFD",
                #         i_DDLY    = d_delayed,
                #         i_CE1     = 1,
                #         i_RST     =  ResetSignal("adc8_frame"),
                #         i_CLK     =  ClockSignal("adc"),
                #         i_CLKB    = ~ClockSignal("adc"),
                #         i_CLKDIV  =  ClockSignal("adc8_frame"),
                #         i_BITSLIP = bitslip,
                #          **{f"o_Q{n+1}": d[8-1-n] for n in range(8)},
                #     )
                # ]
                # self.comb += adc_source.data[8*i:8*(i+1)].eq(d if lanes_polarity[i] == 0 else (d ^ 0xff))

        if pads is None:
            self.adc_source = adc_source = stream.Endpoint([("data", nchannels*16)])
            self.comb += adc_source.valid.eq(1)
            # Generate a Ramp Pattern when no pads are provided.
            for i in range(nchannels):
                adc_data = Signal(8)
                self.sync += adc_data.eq(adc_data + nchannels)
                self.sync += adc_source.data[8*i:8*(i+1)].eq(adc_data + i)


        # Clock Domain Crossing.
        # ----------------------

        # self.submodules.cdc = stream.ClockDomainCrossing(
        #     layout   = [("data", nchannels*16)],
        #     cd_from  = "adc_frame_mux" if pads is not None else clock_domain,
        #     cd_to    = clock_domain,
        #     buffered = True,
        #     depth    = 512
        # )
        # self.comb += self.adc_source.connect(self.cdc.sink)

        # Shuffler.
        # ---------

        # Data from the HMCAD1520 comes in the order [1 1 2 2 3 3 4 4] for 4-channel operation
        #  and [1 1 1 1 2 2 2 2] for 2-channel operation.  Shuffle the data so the output data
        #  stream is [1 2 3 4 1 2 3 4] and [1 2 1 2 1 2 1 2] respectively.
        
        self.adc_shuffler = ByteShuffler(3, byte_swap=[[0, 1, 2, 3, 4, 5, 6, 7],
                                                         [0, 4, 1, 5, 2, 6, 3, 7],
                                                         [0, 2, 4, 6, 1, 3, 5, 7]])
        self.comb += self.adc_shuffler.shuffle.eq(self._data_channels.fields.shuffle)
        self.comb += self.adc_source.connect(self.adc_shuffler.sink)

        # DownSampling.
        # -------------

        self.submodules.downsampling = DownSampling(ratio=self._downsampling.storage)
        self.comb += self.adc_shuffler.source.connect(self.downsampling.sink)
        self.comb += self.downsampling.source.connect(source)
        self.comb += self.downsampling.source.ready.eq(1) # No backpressure allowed.

        # Statistics.
        # -----------

        # Min/Max Range.
        for adc in ["01", "23"]:
            adc_min   = getattr(self._range.fields, f"min{adc}")
            adc_max   = getattr(self._range.fields, f"max{adc}")
            adc_value = {"01": source.data[:8], "23": source.data[-8:]}[adc]
            self.sync += [
                # On a valid cycle:
                If(source.valid,
                    # Compute Min.
                    If(adc_value >= adc_max,
                        adc_max.eq(adc_value)
                    ),
                    # Compute Max.
                    If(adc_value <= adc_min,
                        adc_min.eq(adc_value)
                    )
                ),
                # Clear Min/Max.
                If(self._control.fields.stat_rst,
                    adc_min.eq(0xff),
                    adc_max.eq(0x00)
                ),
            ]

        # if pads is not None:
        #     # BitSlips Count.
        #     bitslip_count = self._bitslip_count.status
        #     self.sync.adc8_frame += bitslip_count.eq(bitslip_count + self.bitslip)

        # Samples Count.
        sample_count = self._sample_count.status
        self.sync += [
            # On a valid cycle:
            If(source.valid,
                If(sample_count != (2**32-nchannels),
                    sample_count.eq(sample_count + nchannels)
                )
            ),
            # Clear Count.
            If(self._control.fields.stat_rst,
                sample_count.eq(0),
            ),
        ]

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class HMCAD15XXADCDriver:
    def __init__(self, bus, spi, n, mode="single"):
        assert mode in ["single", "dual"]
        self.bus     = bus
        self.spi     = spi
        self.n       = n
        self.mode    = mode

        self.control       = getattr(bus.regs, f"adc_had1511_control")
        self.downsampling  = getattr(bus.regs, f"adc_had1511_downsampling")
        self.range         = getattr(bus.regs, f"adc_had1511_range")
        self.bitslip_count = getattr(bus.regs, f"adc_had1511_bitslip_count")
        self.sample_count  = getattr(bus.regs, f"adc_had1511_sample_count")

    def reset(self):
        # Reset ADC.
        self.set_reg(0x00, 0x0001) # Soft Reset.
        time.sleep(0.1)
        self.set_reg(0x0f, 0x2000) # Power-Down.
        time.sleep(0.1)
        self.set_reg(0x0f, 0x0000) # Power-On.

        # Reset Core.
        self.control.write(HAD1511_CORE_CONTROL_FRAME_RST)

    def set_reg(self, reg, value):
        print(f"hmcad 0x{reg:02X} 0x{value:04X}")
        self.spi.write(0, [reg, (value >> 8) & 0xff, value & 0xff])

    def set_gain(self, gain):
        if self.mode == "single":
            self.set_reg(0x2b, (gain << 8))
        if self.mode == "dual":
            self.set_reg(0x2b, (gain << 4) | (gain << 0)) # Note: Apply similar gains on the two channels.

    def set_clk_divider(self, divider=1):
        self.set_reg()

    def data_mode(self, n):
        mode = "dual"
        if isinstance(n, int):
            n = [n, n]
            mode = "single"
        if isinstance(n, list) and (len(n) == 1):
            n = n + n
            mode = "single"
        assert len(n) == 2
        reg31 = {
            "single": 0x0001, # Single Channel ADC0..3 Interleaving / X1 Divider.
            "dual"  : 0x0102, # Dual Channel ADC0..1/ADC2..3 Interleaving / X2 Divider.
        }[mode]
        self.set_reg(0x0f, 0x0200) # Power-Down.
        self.set_reg(0x31,  reg31) # Apply Mode.
        self.set_reg(0x0f, 0x0000) # Power-Up.

        self.set_reg(0x25, 0x0000) # Disable Patterns.

        self.set_reg(0x30, 0x0008) # Clk Jitter Adjustement.
        inp_sel_adc01 = (1 << ((n[0]%2)*3 + 1))
        inp_sel_adc23 = (1 << ((n[1]%2)*3 + 1))
        self.set_reg(0x3a, (inp_sel_adc01 << 8) | inp_sel_adc01) # Connect Input to ADC0/1.
        self.set_reg(0x3b, (inp_sel_adc23 << 8) | inp_sel_adc23) # Connect Input to ADC2/3.
        self.set_reg(0x33, 0x0001) # Coarse Gain in X mode.
        self.set_reg(0x2a, 0x2222) # X2 Gain on all ADCs.

    def enable_ramp_pattern(self):
        self.set_reg(0x25, 0x0040) # Enable Full-Scale Ramp.

    def enable_single_pattern(self, pattern):
        self.set_reg(0x26, pattern) # Set Pattern.
        self.set_reg(0x25, 0x0010)  # Enable Single Pattern.

    def enable_dual_pattern(self, patterns):
        assert len(patterns) == 2
        self.set_reg(0x26, patterns[0]) # Set First Pattern.
        self.set_reg(0x27, patterns[1]) # Programm Second Pattern.
        self.set_reg(0x25, 0x0020)      # Enable Dual Pattern.

    def enable_deskew_pattern(self):
        self.set_reg(0x25, 0x0000) # Disable Patterns.
        self.set_reg(0x45, 0x0002) # Enable Deskew Pattern.

    def enable_sync_pattern(self):
        self.set_reg(0x25, 0x0000) # Disable Patterns.
        self.set_reg(0x45, 0x0001) # Enable Deskew Pattern.

    def get_range(self, n, duration=0.5):
        self.control.write(HAD1511_CORE_CONTROL_STAT_RST)
        time.sleep(duration)
        adc_range = self.range.read()
        adc_min = (adc_range >> ((n%2)*16 + 0)) & 0xff
        adc_max = (adc_range >> ((n%2)*16 + 8)) & 0xff
        return adc_min, adc_max

    def get_samplerate(self, duration=0.5):
        self.control.write(HAD1511_CORE_CONTROL_STAT_RST)
        time.sleep(duration)
        sample_count = self.sample_count.read()
        return sample_count/duration

class HAD1511DMADriver:
    def __init__(self, bus, n):
        self.bus     = bus
        self.n       = n

        self.dma_enable = getattr(bus.regs, f"adc{n}_dma_enable")
        self.dma_base   = getattr(bus.regs, f"adc{n}_dma_base")
        self.dma_length = getattr(bus.regs, f"adc{n}_dma_length")
        self.dma_done   = getattr(bus.regs, f"adc{n}_dma_done")

    def reset(self):
        self.dma_enable.write(0)


    def start(self, base, length):
        self.dma_enable.write(0)
        self.dma_base.write(base)
        self.dma_length.write(length + 1024) # FIXME: +1024.
        self.dma_enable.write(1)

    def wait(self):
        while not (self.dma_done.read() & 0x1):
            pass
