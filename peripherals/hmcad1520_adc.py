#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2020-2021 Felix Domke <tmbinc@elitedvb.net>
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2025 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

import time

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import BusSynchronizer, PulseSynchronizer

from litex.gen import *

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

HMCAD1511_CORE_CONTROL_FRAME_RST = (1 << 0)
HMCAD1511_CORE_CONTROL_DELAY_RST = (1 << 1)
HMCAD1511_CORE_CONTROL_DELAY_INC = (1 << 2)
HMCAD1511_CORE_CONTROL_STAT_RST  = (1 << 3)

HMCAD1511_CORE_RANGE_STAT_MIN    = (1 << 0)
HMCAD1511_CORE_RANGE_STAT_MAX    = (1 << 8)

# ADC.

HMCAD1511_ADC_GAIN_0DB = 0
HMCAD1511_ADC_GAIN_2DB = 2
HMCAD1511_ADC_GAIN_4DB = 4
HMCAD1511_ADC_GAIN_6DB = 6
HMCAD1511_ADC_GAIN_9DB = 9

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# Layouts ------------------------------------------------------------------------------------------

hmcad1520_phy_layout = ["fclk_p", "fclk_n", "lclk_p", "lclk_n", "d_p", "d_n"]

# HMCAD1520 ADC --------------------------------------------------------------------------------------

class HMCAD1520ADC(LiteXModule):
    def __init__(self, pads, sys_clk_freq, frame_polarity=0, lanes_polarity=[0]*8, clock_domain="sys"):
        # Parameters.
        if pads is not None:
            nchannels = len(pads.d_p)
            for name in hmcad1520_phy_layout:
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
            CSRField("stat_rst",  offset=3, size=1, pulse=True, description="Statistics reset."),
            CSRField("data_delay_inc", offset=4, size=1, pulse=True, description="Sampling delay increment."),
        ])
        self._status       = CSRStatus() # Unused (for now).
        self._downsampling = CSRStorage(32, description="ADC Downsampling ratio.")
        self._range        = CSRStatus(fields=[
            CSRField("min01", size=8, offset= 0, description="ADC0/1 Min value since last stat_rst."),
            CSRField("max01", size=8, offset= 8, description="ADC0/1 Max value since last stat_rst."),
            CSRField("min23", size=8, offset=16, description="ADC2/3 Min value since last stat_rst."),
            CSRField("max23", size=8, offset=24, description="ADC2/3 Max value since last stat_rst."),
        ])
        self._bitslip_count = CSRStatus(32, description="ADC bitslip count (with rollover).")
        self._sample_count  = CSRStatus(32, description="ADC samples count since last stat_rst.")
        self._data_channels = CSRStorage(fields=[
            CSRField("shuffle",     offset=0, size=4, reset=0, description="Number of enabled channels to control shuffling of samples",
                    values=[
                        (0, "1-Channel 8-bit Shuffling"),
                        (1, "2-Channel 8-bit Shuffling"),
                        (2, "4-Channel 8-bit Shuffling"),
                        (3, "1-Channel 12-bit Shuffling"),
                        (4, "2-Channel 12-bit Shuffling"),
                        (5, "4-Channel 12-bit Shuffling")
                    ]),
            CSRField("run_length",  offset=8, size=6, reset=1, description="Control Run-Length of samples for each channel ordered next to each other (Not Implemented)")            
        ])
        self._sample_bits = CSRStorage(fields=[
            CSRField("data_width",offset=0, size=2, reset=0, description="Select the ADC Sample data width.",
                    values=[
                        (0b00, "8-bit"),
                        (0b01, "12-bit")
                    ])
        ])
        self._frame_debug = CSRStatus(fields=[
            CSRField("frame_clk",          offset=0, size=8, description="Frame Alignment Clock Pattern"),
            CSRField("frame_valid",             offset=8, size=1, description="Frame Valid signal")
        ])
        self._adc_debug = CSRStatus(fields=[
            CSRField("gearbox_0_valid",     offset=0, size=1),
            CSRField("gearbox_1_valid",     offset=1, size=1),
            CSRField("gearbox_2_valid",     offset=2, size=1),
            CSRField("gearbox_3_valid",     offset=3, size=1),
            CSRField("gearbox_4_valid",     offset=4, size=1),
            CSRField("gearbox_5_valid",     offset=5, size=1),
            CSRField("gearbox_6_valid",     offset=6, size=1),
            CSRField("gearbox_7_valid",     offset=7, size=1)
        ])

        # # #


        # Clocking.
        # ---------

        if pads is not None:
            self.clock_domains.cd_adc       = ClockDomain() # ADC Bitclock.
            self.clock_domains.cd_adc_frame = ClockDomain() # ADC Frameclock (freq : ADC Bitclock/4).
            adc_clk = Signal()
            self.specials += Instance("IBUFDS",
                i_I  = pads.lclk_p,
                i_IB = pads.lclk_n,
                o_O  = adc_clk
            )
            self.specials += Instance("BUFIO",
                i_I = adc_clk,
                o_O = ClockSignal("adc")
            )
            self.specials += Instance("BUFR",
                p_BUFR_DIVIDE = "4",
                i_I = adc_clk,
                o_O = ClockSignal("adc_frame")
            )
            self.specials += AsyncResetSynchronizer(self.cd_adc_frame, self._control.fields.frame_rst)

        # LVDS Reception & Deserialization.
        # ---------------------------------

        if pads is not None:
            self.bitslip        =  bitslip      = Signal()
            self.fclk           = fclk          = Signal(8)
            self.fclk_no_delay  = fclk_no_delay = Signal()
            self.fclk_delayed   = fclk_delayed  = Signal()
            self.frame_valid    = frame_valid   = Signal(reset=0) # Indicates Data framed correctly
            self.width_bits     = width_bits    = Signal(2)

            self.width_synchro = BusSynchronizer(width=2, idomain="sys", odomain="adc_frame")
            self.sync += self.width_synchro.i.eq(self._sample_bits.fields.data_width)
            self.comb += width_bits.eq(self.width_synchro.o)

            # Receive & Deserialize Frame clock to use it as a delimiter for the data.
            self.specials += [
                Instance("IBUFDS",
                    i_I  = pads.fclk_p,
                    i_IB = pads.fclk_n,
                    o_O  = fclk_no_delay
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

                    i_IDATAIN  = fclk_no_delay,
                    o_DATAOUT  = fclk_delayed
                ),
                Instance("ISERDESE2",
                    p_DATA_WIDTH     = 8,
                    p_DATA_RATE      = "DDR",
                    p_SERDES_MODE    = "MASTER",
                    p_INTERFACE_TYPE = "NETWORKING",
                    p_NUM_CE         = 1,
                    p_IOBDELAY       = "IFD",
                    i_DDLY    = fclk_delayed,
                    i_CE1     = 1,
                    i_RST     =  ResetSignal("adc_frame"),
                    i_CLK     =  ClockSignal("adc"),
                    i_CLKB    = ~ClockSignal("adc"),
                    i_CLKDIV  =  ClockSignal("adc_frame"),
                    i_BITSLIP = bitslip,
                     **{f"o_Q{n+1}": fclk[8-1-n] for n in range(8)},
                )
            ]

            # Check Frame clock synchronization and increment bitslip every 1 ms when not synchronized.
            fclk_timer = WaitTimer(int(1e-3*125e6))
            fclk_timer = ClockDomainsRenamer("adc_frame")(fclk_timer)
            self.submodules += fclk_timer
            
            if(frame_polarity):
                frame_valid_8 = 0xf0
                frame_valid_12 = [0xc0, 0x0f, 0xfc]
            else:
                frame_valid_8 = 0x0f
                frame_valid_12 = [0x3f, 0xf0, 0x03]

            self.sync.adc_frame += [
                bitslip.eq(0),
                fclk_timer.wait.eq(~fclk_timer.done),
                If(fclk_timer.done,
                    If((width_bits == 0b0) & (fclk != frame_valid_8),
                        bitslip.eq(1)
                    ).Elif((width_bits == 0b1) & ((fclk != frame_valid_12[0])
                            & (fclk != frame_valid_12[1]) & (fclk != frame_valid_12[2])),
                        bitslip.eq(1)
                    )
                )
            ]

            # Start of Frame Alignment FSM.
            frame_fsm = FSM(reset_state="IDLE")
            self.submodules.frame_fsm = frame_fsm = ClockDomainsRenamer("adc_frame")(frame_fsm)

            frame_fsm.act("IDLE",
                NextValue(frame_valid, 0),
                NextState("DETECT0")
            )
            frame_fsm.act("DETECT0",
                # Start of Frame detected.
                NextValue(frame_valid, 0),
                NextState("DETECT0"),
                # Wait for the next frame clock to be valid.
                If((width_bits == 0b0) & (fclk == frame_valid_8),
                    NextValue(frame_valid, 1),
                    NextState("SYNC")
                ).Elif((width_bits == 0b1) & (fclk == frame_valid_12[0]),
                    NextState("DETECT1")
                )
            )
            frame_fsm.act("DETECT1",
                # Start of Frame detected.
                NextValue(frame_valid, 0),
                # Wait for the next frame clock to be valid.
                If((width_bits == 0b1) & (fclk == frame_valid_12[1]),
                    NextState("DETECT2")
                ).Else(
                    # If the next frame clock is not valid, wait for the next one.
                    NextState("DETECT0")
                )
            )
            frame_fsm.act("DETECT2",
                # Start of Frame detected.
                NextValue(frame_valid, 0),
                # Wait for the next frame clock to be valid.
                If((width_bits == 0b1) & (fclk == frame_valid_12[2]),
                    NextValue(frame_valid, 1),
                    NextState("SYNC")
                ).Else(
                    # If the next frame clock is not valid, wait for the next one.
                    NextState("DETECT0")
                )
            )
            frame_fsm.act("SYNC",
                If(bitslip,
                    NextValue(frame_valid, 0),
                    NextState("DETECT0")
                ).Elif((width_bits == 0b0) & (fclk != frame_valid_8),
                    NextValue(frame_valid, 0),
                    NextState("DETECT0")
                ).Elif((width_bits == 0b1) & ((fclk != frame_valid_12[0])
                        & (fclk != frame_valid_12[1]) & (fclk != frame_valid_12[2])),
                    NextValue(frame_valid, 0),
                    NextState("DETECT0")
                ).Else(
                    NextValue(frame_valid, 1),
                    NextState("SYNC")
                )
            )
            
            adc_valid_synchro = BusSynchronizer(width=9, idomain="adc_frame", odomain=clock_domain)
            self.sync.adc_frame += [
                adc_valid_synchro.i[0:8].eq(fclk),
                adc_valid_synchro.i[8].eq(frame_valid)
            ]
            self.submodules += adc_valid_synchro

            self.comb += [
                self._frame_debug.fields.frame_clk.eq(adc_valid_synchro.o[0:8]),
                self._frame_debug.fields.frame_valid.eq(adc_valid_synchro.o[8])
            ]

            # Receive & Deserialize Data.
            self.adc_source = adc_source = stream.Endpoint([("data", nchannels*16)])

            stat_rst = PulseSynchronizer(idomain="sys", odomain="adc_frame")
            self.comb += stat_rst.i.eq(self._control.fields.stat_rst)


            for i in range(nchannels):
                d_no_delay = Signal()
                d_delayed  = Signal()
                d          = Signal(8)

                self.specials += [
                    Instance("IBUFDS",
                        i_I  = pads.d_p[i],
                        i_IB = pads.d_n[i],
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
                        i_LD       = self._control.fields.delay_rst,
                        i_CE       = self._control.fields.data_delay_inc,
                        i_LDPIPEEN = 0,
                        i_INC      = 1,

                        i_IDATAIN  = d_no_delay,
                        o_DATAOUT  = d_delayed
                    ),
                    Instance("ISERDESE2",
                        p_DATA_WIDTH     = 8,
                        p_DATA_RATE      = "DDR",
                        p_SERDES_MODE    = "MASTER",
                        p_INTERFACE_TYPE = "NETWORKING",
                        p_NUM_CE         = 1,
                        p_IOBDELAY       = "IFD",
                        i_DDLY    = d_delayed,
                        i_CE1     = 1,
                        i_RST     =  ResetSignal("adc_frame"),
                        i_CLK     =  ClockSignal("adc"),
                        i_CLKB    = ~ClockSignal("adc"),
                        i_CLKDIV  =  ClockSignal("adc_frame"),
                        i_BITSLIP = bitslip,
                         **{f"o_Q{n+1}": d[8-1-n] for n in range(8)},
                    )
                ]

                # If 8-bit mode, pack 2 samples into 16 bits.
                d8_gear = stream.Gearbox(i_dw=8, o_dw=16, msb_first=False)
                d8_gear = ClockDomainsRenamer("adc_frame")(d8_gear)

                d12_gear = stream.Gearbox(i_dw=8, o_dw=12, msb_first=False)
                d12_gear = ClockDomainsRenamer("adc_frame")(d12_gear)
                d12_gear = ResetInserter()(d12_gear)

                self.submodules += d8_gear
                self.submodules += d12_gear

                self.comb += [
                    d8_gear.sink.data.eq(d if lanes_polarity[i] == 0 else (d ^ 0xff)),
                    d12_gear.sink.data.eq(d if lanes_polarity[i] == 0 else (d ^ 0xff)),
                    d12_gear.reset.eq(~frame_valid),
                    d8_gear.source.ready.eq(1),
                    d12_gear.source.ready.eq(1),
                    d8_gear.sink.valid.eq(frame_valid),
                    d12_gear.sink.valid.eq(frame_valid)
                ]

                gearbox_v = BusSynchronizer(width=1, idomain="adc_frame", odomain=clock_domain)
                self.comb += gearbox_v.i.eq(d12_gear.source.valid)
                self.comb += self._adc_debug.fields.__getattribute__(f"gearbox_{i}_valid").eq(gearbox_v.o)

                gearbox_cases = {}
                gearbox_cases[0] = [
                    adc_source.data[16*i:16*(i+1)].eq(d8_gear.source.data),
                    adc_source.valid.eq(d8_gear.source.valid),
                ]
                gearbox_cases[1] = [
                    # adc_source.data[16*i:((16*i)+12)].eq(d12_gear.source.data),
                    adc_source.data[((16*i)+4):(16*(i+1))].eq(d12_gear.source.data),
                    # Sign-extend the 12-bit data to 16 bits.
                    # {adc_source.data[(16*i)+12+j].eq(d12_gear.source.data[11]) for j in range(4)},
                    {adc_source.data[(16*i)+j].eq(0) for j in range(4)},
                    adc_source.valid.eq(d12_gear.source.valid),
                ]

                self.sync.adc_frame += Case(width_bits, gearbox_cases)


        # Clock Domain Crossing.
        # ----------------------

        self.cdc = stream.ClockDomainCrossing(
            layout   = [("data", nchannels*16)],
            cd_from  = "adc_frame",
            cd_to    = clock_domain,
            buffered = True,
            depth=8
        )

        # Shuffler.
        # ---------

        # Data from the HMCAD1520 comes in the order [1 1 2 2 3 3 4 4] for 4-channel operation
        #  and [1 1 1 1 2 2 2 2] for 2-channel operation.  Shuffle the data so the output data
        #  stream is [1 2 3 4 1 2 3 4] and [1 2 1 2 1 2 1 2] respectively.
        self.adc_shuffler = ByteShuffler(6, byte_swap=[ [0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15],
                                                        [0, 8, 2, 10, 4, 12, 6, 14, 1, 9, 3, 11, 5, 13, 7, 15],
                                                        [0, 4, 8, 12, 2, 6, 10, 14, 1, 5, 9, 13, 3, 7, 11, 15],
                                                        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
                                                        [0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15],
                                                        [0, 1, 4, 5, 8, 9, 12, 13, 2, 3, 6, 7, 10, 11, 14, 15]
                                                    ])
        self.comb += self.adc_shuffler.shuffle.eq(self._data_channels.fields.shuffle)

        # DownSampling.
        # # -------------
        # self.submodules.downsampling = DownSampling(ratio=self._downsampling.storage)

        self.submodules += stream.Pipeline(
                                    self.adc_source,
                                    self.cdc,
                                    self.adc_shuffler,
                                    self.source
                                )
        
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

        if pads is not None:
            # BitSlips Count.
            self.bitslip_synchro = PulseSynchronizer(idomain="adc_frame", odomain=clock_domain)
            self.sync.adc_frame += self.bitslip_synchro.i.eq(self.bitslip)
            bitslip_count = self._bitslip_count.status
            self.sync += self._bitslip_count.status.eq(bitslip_count + self.bitslip_synchro.o)

        # Samples Count.
        sample_count = self._sample_count.status
        self.sync += [
            # On a valid cycle:
            If(source.valid,
                If(sample_count != (2**32-nchannels),
                   If(self._sample_bits.fields.data_width == 0b00, # 8-bit
                        sample_count.eq(sample_count + (nchannels*2)),
                    ).Else( # 12-bit
                        sample_count.eq(sample_count + nchannels)
                    )
                )
            ),
            # Clear Count.
            If(self._control.fields.stat_rst,
                sample_count.eq(0),
            ),
        ]

