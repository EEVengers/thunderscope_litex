#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2025 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import MultiReg
from migen.genlib.fifo import AsyncFIFOBuffered

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

event_layout = [("data", 64), ("type", 4), ("reserved", 4)]

_EVENT_IN_OUT_MAX = 12

class EventFIFO(LiteXModule):
    def __init__(self):
        self.eventFifo = AsyncFIFOBuffered(width = 72, depth=1024)
        self.eventData = stream.Endpoint(event_layout)

        self._readport = CSRStatus(8, description="Event Source ID")
        self._readmarker = CSRStatus(64, description="Sample Counter where event ocurred")

        # Input from signal gets pushed to fifo

        # Register reads output from FIFO
        # FIFO width is > 32 bit, register both values when the first one is read

class EventEngine(LiteXModule):
    def __init__(self, marker=None):
        self.en = Signal() # Global Event Enable
        self._inputs = []
        self._outputs = []
        self._marker = Signal.like(marker)

        self._control = CSRStorage(fields=[
            CSRField("in_en_mask", offset=0, size=_EVENT_IN_OUT_MAX, description="Mask for enabled Input Signals."),
            CSRField("out_en_mask", offset=16, size=_EVENT_IN_OUT_MAX, description="Mask for enabled Output Signals"),
            CSRField("event_flush", offset=31, size=1, pulse=True, description="Clear any pending events in the Event FIFO")
        ])

        self._status = CSRStatus(fields=[
            CSRField("in_stat", offset=0, size=_EVENT_IN_OUT_MAX, description="Bitfield of current Event Input signals")
        ])

        self._event = CSRStatus(fields = [
            CSRField("pending", offset=0, size=1, description="Pending Event Ready")
        ])

        # self.submodules.fifo = _fifo = EventFIFO()



    def add_input(self, input):
        self._inputs.append(input)

    def add_output(self, output):
        self._outputs.append(output)

    def map_events(self):
        # TODO
        pass
    
class EventGenerator(LiteXModule):
    def __init__(self, sys_clk_freq=100e6):
        self.event = Signal()
        # Register interface that triggers an event signal
        self._control = CSRStorage(fields=[
            CSRField("immediate", offset=0, size=1, pulse=True, description="Indicate an Immediate Event"),
            CSRField("periodic", offset=1, size=1, pulse=False, reset=0, description="Enable Periodic Event generation")
        ])
        self._timeout = CSRStorage(32, reset=0, description="Event Trigger Period (us)")

        # Event can be set immediately or periodically on a timer
        evt_trigger = Signal()
        evt_periodic_valid = Signal()
        evt_counter = Signal(32)
        evt_timer = WaitTimer(int((1e-6)*sys_clk_freq)) # 1us Timer

        self.comb += evt_periodic_valid.eq(self._control.fields.periodic & (self._timeout.storage > 0))

        self.sync += [
            evt_trigger.eq(evt_periodic_valid & (evt_counter == 0)),
            evt_timer.wait.eq(~evt_timer.done & evt_periodic_valid),
            If(evt_timer.done,
                If(evt_counter == 0,
                   evt_counter.eq(self._timeout.storage - 1)
                ).Else(
                    evt_counter.eq(evt_counter - 1)
                )
            ),
            self.event.eq(
                self._control.fields.immediate |
                evt_trigger
            )
        ]

class ExternalSync(LiteXModule):
    def __init__(self, pads=None, sys_clk_freq=100e6):
        self.ext_in = Signal()
        self.ext_out = Signal()

        _in_unfiltered = Signal()
        _out_pulse = Signal()
        _ext_out_last = Signal()

        self._control  = CSRStorage(2, description="Sync Tristate(s) Control. Valid Values are:"\
                                        "\n\t0b00 - Disabled" \
                                        "\n\t0b01 - Input Enabled" \
                                        "\n\t0b10 - Output Enabled"
                                    )
        self._pulse_len = CSRStorage(20, reset=50,
                                     description="Pulse Width of the Sync Output Signal in microseconds (us). Default is 50us")
        self._status  = CSRStatus(fields=[
            CSRField("evt_in", offset=0, size=1,  description="Sync Input Status."),
            CSRField("evt_out", offset=8, size=1,  description="Sync Output Status.")
        ])
        
        self.comb += [
            self._status.fields.evt_in.eq(self.ext_in),
            self._status.fields.evt_out.eq(self.ext_out)
        ]

        # Synchronize Input Signal
        self.specials += MultiReg(i=_in_unfiltered, o=self.ext_in)

        # Set Output Pulse Width
        pulse_counter = Signal(20)
        pulse_timer = WaitTimer(int((1e-6)*sys_clk_freq)) # 1us Timer

        self.sync += [
            pulse_timer.wait.eq(_out_pulse),
            If((_ext_out_last == 0) & self.ext_out, # Rising edge detect
                pulse_counter.eq(self._pulse_len.storage),
                _out_pulse.eq(1),
                pulse_timer.wait.eq(1),
            ),
            If(pulse_timer.done,
                If(pulse_counter == 0,
                   _out_pulse.eq(0),
                   pulse_timer.wait.eq(0),
                ).Else(
                    pulse_counter.eq(pulse_counter - 1),
                   _out_pulse.eq(1)
                )
            ),
            _ext_out_last.eq(self.ext_out)
        ]

        if pads is not None:
            if hasattr(pads, "de"):
                # Dev and Production units have a differential buffer for Sync I/O
                self.specials += [
                    Instance("IBUFDS",
                        i_I  = pads.in_p,
                        i_IB = pads.in_n,
                        o_O  = _in_unfiltered
                    ),
                    Instance("OBUFDS",
                        i_I  = _out_pulse,
                        o_O  = pads.out_p,
                        o_OB = pads.out_n,
                    )
                ]

                self.comb += [
                    If(self._control.storage == 0b01,
                       pads.re_n.eq(0)
                    ).Else(
                        pads.re_n.eq(1)
                    ),
                    If(self._control.storage == 0b10,
                       pads.de.eq(1)
                    ).Else(
                        pads.de.eq(0)
                    ),
                ]
            else:
                # This is to support Beta units with a single Tri-state pin sync
                io = TSTriple()
                self.specials += io.get_tristate(pads)
                self.comb += [
                    _in_unfiltered.eq(io.i),
                    io.o.eq(_out_pulse),
                    If(self._control.storage == 0b10,
                       io.oe.eq(1)
                    ).Else(io.io.eq(0))
                ]
