##
# Helper script to format version strings
# https://gist.github.com/eugene-babichenko/f37d15626160914427563dff2edd57ed
##
TAG_COMMIT := $(shell git rev-list --abbrev-commit --tags --max-count=1)
TAG := $(shell git describe --abbrev=0 --tags ${TAG_COMMIT} 2>/dev/null || true)
COMMIT := $(shell git rev-parse --short HEAD)
DATE := $(shell git log -1 --format=%cd --date=format:"%Y%m%d")
BUILD_VERSION := $(TAG:v%=%)
ifneq ($(COMMIT), $(TAG_COMMIT))
	BUILD_VERSION := $(BUILD_VERSION)-next-$(COMMIT)-$(DATE)
endif
ifeq ($(TAG_COMMIT),)
	BUILD_VERSION := $(COMMIT)-$(DATE)
endif
ifneq ($(shell git status --porcelain),)
	BUILD_VERSION := $(BUILD_VERSION)-dirty
endif
