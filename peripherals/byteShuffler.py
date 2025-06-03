

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

# Shuffler -----------------------------------------------------------------------------------------

class ByteShuffler(LiteXModule):
    def __init__(self, num_shuffle, byte_swap = []):
        self.sink   = sink   = stream.Endpoint([("data", len(byte_swap[0])*8)])
        self.source = source = stream.Endpoint([("data", len(byte_swap[0])*8)])

        self.shuffle = Signal(num_shuffle)

        self.comb += sink.connect(source)

        for i in range(len(byte_swap[0])):
            cases = {}            
            for idx, swap in enumerate(byte_swap):
                cases[idx] = self.source.data[i*8:(i+1)*8].eq(self.sink.data[swap[i]*8:(swap[i]+1)*8])

            self.comb += Case(self.shuffle, cases)

