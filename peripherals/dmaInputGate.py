#
# This file is part of Thunderscope-LiteX project.
# SPDX-License-Identifier: BSD-2-Clause

from migen import If, Module, Signal

from litex.soc.interconnect import stream

# Always consume incoming words. Forward only when enabled and DMA is ready.
# Latch (drop_latch) if words discarded while enabled. Clear the latch while disabled.
class DmaInputGate(Module):
    def __init__(self, data_width):
        self.sink = sink = stream.Endpoint([("data", data_width)])
        self.source = source = stream.Endpoint([("data", data_width)])
        self.enable = enable = Signal()
        self.drop_latch = Signal()

        drop_event = Signal()
        drop_history = Signal()
        self.comb += [
            sink.connect(source, omit={"valid", "ready"}),
            sink.ready.eq(1),
            source.valid.eq(enable & sink.valid & source.ready),
            drop_event.eq(enable & sink.valid & ~source.ready),
            self.drop_latch.eq(enable & drop_history),
        ]
        self.sync += [
            If(~enable,
                drop_history.eq(0)
            ).Elif(drop_event,
                drop_history.eq(1)
            )
        ]
