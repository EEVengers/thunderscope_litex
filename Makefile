# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2025 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

include version.mk
export BUILD_VERSION

# Use Python3.  Some systems may need this to be just 'python'
PY:=python3

PROJECT:=thunderscope

LITEX_RELEASE_TAG:=2024.12

# List all build variants
BETA_VARIANTS:= a50t a100t a200t
PROD_VARIANTS:= dev prod

# Define the set of supported variants to be built as part of a release
RELEASE_VARIANTS= $(PROD_VARIANTS)

# Collect all possible variants
ALL_VARIANTS=$(RELEASE_VARIANTS) $(BETA_VARIANTS)

# LiteX Generate-only targets
GEN_VARIANTS=$(patsubst %,gen-%, $(ALL_VARIANTS))

# Paths to use for building
BUILD_PATH:= build
DIST_PATH:= distrib

# Source files to pickup changes from
SRC_PATH:= . peripherals
SOURCES:= $(foreach x, $(SRC_PATH), $(wildcard $(addprefix $(x)/*,.py*)))

## Configure Virtual Environment
VENV_PATH:=.venv

ifeq ($(OS),Windows_NT)
VENV_ACTIVATE:=$(VENV_PATH)/activate.bat
else
VENV_ACTIVATE:=. $(VENV_PATH)/bin/activate
endif

.PHONY: venv
$(VENV_PATH)/.stamp_$(LITEX_RELEASE_TAG):
	@rm -rf $(VENV_PATH)
	$(PY) -m venv $(VENV_PATH)
	$(VENV_ACTIVATE) && \
	pip install --upgrade pip && \
	pip install -r requirements.txt && \
	mkdir $(VENV_PATH)/litex && \
	cd $(VENV_PATH)/litex && \
	wget https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py && \
	$(PY) litex_setup.py --init --install --tag $(LITEX_RELEASE_TAG)
	touch $(VENV_PATH)/.stamp_$(LITEX_RELEASE_TAG)

venv: $(VENV_PATH)/.stamp_$(LITEX_RELEASE_TAG)
	@echo "Using virtual environment, LiteX version: $(LITEX_RELEASE_TAG)"

.PHONY: all release driver docs

# Release target to build supported artifacts
release: $(RELEASE_VARIANTS) docs driver
	@cp -a $(BUILD_PATH)/$(PROJECT)/doc $(DIST_PATH)/doc


all: $(ALL_VARIANTS) docs driver


$(ALL_VARIANTS) : $(SOURCES) venv
	$(VENV_ACTIVATE) && \
	$(PY) $(PROJECT).py --variant=$@ --build --output-dir=$(BUILD_PATH)/$(PROJECT)_$@
	# Copy to destination folder
	@mkdir -p $(DIST_PATH)/$@
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_full.bin $(DIST_PATH)/$@/$(PROJECT)_full_$@_$(BUILD_VERSION).bin
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_full.mcs $(DIST_PATH)/$@/$(PROJECT)_full_$@_$(BUILD_VERSION).mcs
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_update.bin $(DIST_PATH)/$@/$(PROJECT)_update_$@_$(BUILD_VERSION).bin
	@cp $(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_update.bit $(DIST_PATH)/$@/$(PROJECT)_update_$@_$(BUILD_VERSION).bit


.PHONY: gen $(GEN_VARIANTS)
gen: $(GEN_VARIANTS) venv

$(GEN_VARIANTS): gen-% : $(SOURCES)
	$(VENV_ACTIVATE) && \
	$(PY) $(PROJECT).py --variant=$* --output-dir=$(BUILD_PATH)/$(PROJECT)_$*


driver: venv
	$(VENV_ACTIVATE) && \
	$(PY) $(PROJECT).py --driver --driver-dir=$(BUILD_PATH)/$(PROJECT)/driver

docs: venv
	$(VENV_ACTIVATE) && \
	$(PY) $(PROJECT).py --variant=prod --doc
	@sphinx-build $(BUILD_PATH)/$(PROJECT)/doc $(BUILD_PATH)/docs
	zip -ur $(DIST_PATH)/$(PROJECT)_docs.zip $(BUILD_PATH)/docs/*

distclean:
	@rm -rf $(BUILD_PATH)/*
	@rm -rf $(DIST_PATH)
