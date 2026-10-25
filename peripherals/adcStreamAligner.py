#
# This file is part of Thunderscope-LiteX project.
# SPDX-License-Identifier: BSD-2-Clause

from migen import Constant

from litex.gen import LiteXModule
from litex.soc.interconnect import stream

class AdcStreamAligner(LiteXModule):
    def __init__(self, data_width):
        self.sink = stream.Endpoint([("data", data_width)])
        self.source = stream.Endpoint([("data", data_width)])
        self._side_channels = []
        self.comb += self.sink.connect(self.source, omit={"valid", "ready"})

    def add_side_channel(self, data_width):
        sink = stream.Endpoint([("data", data_width)])
        source = stream.Endpoint([("data", data_width)])
        self._side_channels.append((sink, source))
        self.comb += sink.connect(source, omit={"valid", "ready"})
        return sink, source

    def do_finalize(self):
        sinks = [self.sink] + [sink for sink, _ in self._side_channels]
        valid = Constant(1)
        for sink in sinks:
            valid = valid & sink.valid
        self.comb += self.source.valid.eq(valid)
        for sink in sinks:
            self.comb += sink.ready.eq(valid & self.source.ready)
        # Side-channel consumers observe transfers, not stalled ADC words.
        for _, source in self._side_channels:
            self.comb += source.valid.eq(valid & self.source.ready)
