# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2025 Nate Meyer <nate.devel@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

include version.mk
export BUILD_VERSION

export PYTHONUTF8:=1

# Use the native shell on Windows; no Unix command-line utilities are required.
ifeq ($(OS),Windows_NT)
SHELL:=cmd.exe
.SHELLFLAGS:=/c
PY?=py -V:3.12
else
PY?=python3
endif

PROJECT:=thunderscope

LITEX_RELEASE_TAG:=2024.12
export LITEX_RELEASE_TAG

# List all build variants
BETA_VARIANTS:= a50t a100t a200t
PROD_VARIANTS:= dev prod

# Define the set of supported variants to be built as part of a release
RELEASE_VARIANTS= $(PROD_VARIANTS)

# Collect all possible variants
ALL_VARIANTS=$(RELEASE_VARIANTS) $(BETA_VARIANTS)

# LiteX Generate-only targets
GEN_VARIANTS=$(patsubst %,gen-%, $(ALL_VARIANTS))

# LiteX JTAG Load-only targets
LOAD_VARIANTS=$(patsubst %,load-%, $(ALL_VARIANTS))

# Paths to use for building
BUILD_PATH:= build
DIST_PATH:= distrib

# Source files to pickup changes from
SRC_PATH:= . peripherals
SOURCES:= $(foreach x, $(SRC_PATH), $(wildcard $(addprefix $(x)/*,.py*)))

## Configure Virtual Environment
VENV_PATH:=.venv
# Avoid shadowing the editable litex package when Python searches the venv root.
LITEX_PATH:=$(VENV_PATH)/litex-src

ifeq ($(OS),Windows_NT)
VENV_PY:=$(subst /,\,$(abspath $(VENV_PATH)/Scripts/python.exe))
VENV_RUN:=set "PATH=$(subst /,\,$(abspath $(VENV_PATH)/Scripts));$(PATH)" && "$(VENV_PY)"
else
VENV_PY:=$(abspath $(VENV_PATH)/bin/python)
VENV_RUN:=PATH="$(abspath $(VENV_PATH)/bin):$$PATH" "$(VENV_PY)"
endif

.PHONY: venv
# Use current repository definitions, but avoid the CLI's auto-update/restart.
$(VENV_PATH)/.stamp_$(LITEX_RELEASE_TAG): Makefile requirements.txt
	@$(PY) -c "import os, pathlib, shutil, stat; p = pathlib.Path('$(VENV_PATH)'); shutil.rmtree(p, onerror=lambda f, path, exc: (os.chmod(path, stat.S_IWRITE), f(path))) if p.exists() else None"
	$(PY) -m venv "$(VENV_PATH)"
	$(VENV_RUN) -m pip install --upgrade pip
	$(VENV_RUN) -m pip install -r requirements.txt
	$(VENV_RUN) -c "from pathlib import Path; Path('$(LITEX_PATH)').mkdir()"
	$(VENV_RUN) -c "import urllib.request; urllib.request.urlretrieve('https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py', '$(LITEX_PATH)/litex_setup.py')"
	cd "$(LITEX_PATH)" && $(VENV_RUN) -c "import litex_setup; litex_setup.litex_setup_import_repos(download=True); litex_setup.litex_setup_init_repos(tag='$(LITEX_RELEASE_TAG)'); litex_setup.litex_setup_install_repos()"
	$(VENV_RUN) -c "import $(PROJECT)"
	$(VENV_RUN) -c "from pathlib import Path; Path('$(VENV_PATH)/.stamp_$(LITEX_RELEASE_TAG)').touch()"

venv: $(VENV_PATH)/.stamp_$(LITEX_RELEASE_TAG)
	@echo "Using virtual environment, LiteX version: $(LITEX_RELEASE_TAG)"

.PHONY: all release driver docs distclean $(ALL_VARIANTS) $(LOAD_VARIANTS)

# Release target to build supported artifacts
release: $(RELEASE_VARIANTS) docs driver
	@$(VENV_RUN) -c "import shutil; shutil.copytree('$(BUILD_PATH)/$(PROJECT)/doc', '$(DIST_PATH)/doc', dirs_exist_ok=True)"


all: $(ALL_VARIANTS) docs driver

$(ALL_VARIANTS) : $(SOURCES) venv
	$(VENV_RUN) $(PROJECT).py --variant=$@ --build --output-dir=$(BUILD_PATH)/$(PROJECT)_$@
	@$(VENV_RUN) -c "from pathlib import Path; Path('$(DIST_PATH)/$@').mkdir(parents=True, exist_ok=True)"
	@$(VENV_RUN) -c "import shutil; shutil.copy2('$(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_full.bin', '$(DIST_PATH)/$@/$(PROJECT)_full_$@_$(BUILD_VERSION).bin')"
	@$(VENV_RUN) -c "import shutil; shutil.copy2('$(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_full.mcs', '$(DIST_PATH)/$@/$(PROJECT)_full_$@_$(BUILD_VERSION).mcs')"
	@$(VENV_RUN) -c "import shutil; shutil.copy2('$(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_update.bin', '$(DIST_PATH)/$@/$(PROJECT)_update_$@_$(BUILD_VERSION).bin')"
	@$(VENV_RUN) -c "import shutil; shutil.copy2('$(BUILD_PATH)/$(PROJECT)_$@/gateware/$(PROJECT)_update.bit', '$(DIST_PATH)/$@/$(PROJECT)_update_$@_$(BUILD_VERSION).bit')"


.PHONY: gen $(GEN_VARIANTS)
gen: $(GEN_VARIANTS) venv

$(GEN_VARIANTS): gen-% : $(SOURCES) venv
	$(VENV_RUN) $(PROJECT).py --variant=$* --gen --output-dir=$(BUILD_PATH)/$(PROJECT)_$*

$(LOAD_VARIANTS): load-% : $(SOURCES) venv
	$(VENV_RUN) $(PROJECT).py --variant=$* --load --output-dir=$(BUILD_PATH)/$(PROJECT)_$*

driver: venv
	$(VENV_RUN) $(PROJECT).py --driver --driver-dir=$(BUILD_PATH)/$(PROJECT)/driver --csr-csv=$(BUILD_PATH)/$(PROJECT)/csr.csv

docs: venv
	$(VENV_RUN) $(PROJECT).py --variant=prod --doc
	$(VENV_RUN) -m sphinx $(BUILD_PATH)/$(PROJECT)/doc $(BUILD_PATH)/docs
	@$(VENV_RUN) -c "from pathlib import Path; Path('$(DIST_PATH)').mkdir(parents=True, exist_ok=True)"
	$(VENV_RUN) -c "import shutil; shutil.make_archive('$(DIST_PATH)/$(PROJECT)_docs', 'zip', '.', '$(BUILD_PATH)/docs')"

distclean:
	@$(PY) -c "from pathlib import Path; import os, shutil, stat; [shutil.rmtree(p, onerror=lambda f, path, exc: (os.chmod(path, stat.S_IWRITE), f(path))) for p in (Path('$(BUILD_PATH)'), Path('$(DIST_PATH)')) if p.exists()]"
