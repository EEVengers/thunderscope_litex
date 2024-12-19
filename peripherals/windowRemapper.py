#
# Wishbone Remapper module extended to include a movable window between the
# source and destination regions.
#
# Copyright (c) 2015-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from math import log2

from migen import *
from litex.gen import *

from litex.soc.interconnect.csr import *

# Wishbone Window Remapper --------------------------------------------------------------------------------
class WindowRemapper(LiteXModule):
    """
    Wishbone Remapper that supports sequential application of:
    - An initial origin offset and address mask.
    - Region-based remapping from specified source regions to destination regions.
    This allows for an initial origin remap followed by more complex/specific region-based remapping.
    """
    def __init__(self, master, slave, origin=0, size=None, src_regions=[], dst_regions=[]):
        # Parameters.
        # -----------
        assert master.addressing == slave.addressing
        assert len(src_regions)  == len(dst_regions)

        # Master to Slave.
        # ----------------
        self.comb += master.connect(slave)

        # Origin Remapping.
        # -----------------
        # Compute Address Mask.
        if size is None:
            size = 2**master.address_width
        log2_size = int(log2(size))
        if master.addressing == "word":
            log2_size -=  int(log2(len(master.dat_w)//8))
            origin    >>= int(log2(len(master.dat_w)//8))
        adr_mask  = 2**log2_size - 1
        # Apply Address Origin/Mask Remapping.
        adr_remap = (origin | (master.adr & adr_mask))
        self.comb += slave.adr.eq(adr_remap)

        # Regions Remapping.
        # ------------------
        # Compute Address Shift.
        adr_shift = {
            "byte" : 0,
            "word" : int(log2(len(master.dat_w)//8)),
        }[master.addressing]
        self.windows = []
        # Apply Address Regions Remapping.
        for src_region, dst_region, idx in zip(src_regions, dst_regions, range(len(src_regions))):
            src_adr = Signal.like(master.adr + adr_shift + 1)
            dst_adr = Signal.like(master.adr + adr_shift + 1)
            active  = Signal()
            reg = CSRStorage(size=32, reset=0, name=f"window{idx}", description=f"Region {idx} Window Offset")
            self.windows.append(reg)
            setattr(self, f"window{idx}", reg)
            self.comb += [
                src_adr.eq(adr_remap << adr_shift),
                dst_adr.eq(dst_region.origin + src_adr - src_region.origin + (reg.storage * src_region.size)),
                active.eq((src_adr >= src_region.origin) & (src_adr < (src_region.origin + src_region.size))),
                If(active, slave.adr.eq(dst_adr >> adr_shift))
            ]
