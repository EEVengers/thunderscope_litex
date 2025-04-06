# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2025 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

# Use Python3.  Some systems may need this to be just 'python'
PY:=python3

PROJECT:=thunderscope

# List all build variants
BETA_VARIANTS:= a50t a100t a200t


# Define the set of supported variants to be built as part of a release
RELEASE_VARIANTS= $(BETA_VARIANTS)

# Collect all possible variants
ALL_VARIANTS= $(RELEASE_VARIANTS) a35t

# Paths to use for building
BUILD_PATH:= build
DIST_PATH:= distrib

# Source files to pickup changes from
SRC_PATH:= . peripherals
SOURCES:= $(foreach x, $(SRC_PATH), $(wildcard $(addprefix $(x)/*,.py*)))


.PHONY: all release driver docs

# Release target to build supported artifacts
release: $(RELEASE_VARIANTS) docs driver
	@cp -a $(BUILD_PATH)/$(PROJECT)/doc $(DIST_PATH)/doc


all: $(ALL_VARIANTS) docs driver


$(ALL_VARIANTS) : $(SOURCES)
	$(PY) $(PROJECT).py --variant=$@ --build --output-dir=$(BUILD_PATH)/$(PROJECT)_$@
	# Copy to destination folder
	@mkdir -p $(DIST_PATH)/$@
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_full.bin $(DIST_PATH)/$@/$(PROJECT)_$@_full.bin
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_full.mcs $(DIST_PATH)/$@/$(PROJECT)_$@_full.mcs
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_update.bin $(DIST_PATH)/$@/$(PROJECT)_$@_update.bin
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_update.bit $(DIST_PATH)/$@/$(PROJECT)_$@_update.bit

driver:
	$(PY) $(PROJECT).py --driver --driver-dir=$(BUILD_PATH)/$(PROJECT)/driver

docs:
	$(PY) $(PROJECT).py --doc
	@sphinx-build $(BUILD_PATH)/$(PROJECT)/doc doc

distclean:
	@rm -rf $(BUILD_PATH)/*
	@rm -rf $(DIST_PATH)
