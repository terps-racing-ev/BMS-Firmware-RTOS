# Host-side unit test build for firmware modules using Unity
# Usage:
#   make -f tests/HostTests.make test
#   make -f tests/HostTests.make test-file FILE=balance_manager
#   make -f tests/HostTests.make clean

CC ?= gcc

TEST_ROOT := tests
BUILD_DIR := $(TEST_ROOT)/build
SUPPORT_DIR := $(TEST_ROOT)/support
INCLUDE_DIR := $(TEST_ROOT)/include
CORE_INC := Core/Inc
CORE_SRC := Core/Src

UNITY_SRC := $(CORE_SRC)/unity.c
SUPPORT_SRCS := $(SUPPORT_DIR)/host_rtos_stub.c \
                $(SUPPORT_DIR)/balance_manager_deps_stub.c

COMMON_INCLUDES := -I$(INCLUDE_DIR) -I$(CORE_INC)
COMMON_CFLAGS := -std=c11 -O0 -g -Wall -Wextra -Wno-unused-parameter -Wno-sign-conversion -Wno-conversion
COMMON_LDFLAGS := -lm

TEST_SRCS := $(wildcard $(TEST_ROOT)/test_*.c)
MODULES := $(patsubst $(TEST_ROOT)/test_%.c,%,$(TEST_SRCS))

.PHONY: test test-file list clean help
.SECONDARY:

help:
	@echo "Host test targets:"
	@echo "  make -f tests/HostTests.make test"
	@echo "  make -f tests/HostTests.make test-file FILE=<module>"
	@echo "  make -f tests/HostTests.make list"
	@echo "  make -f tests/HostTests.make clean"

test: $(addprefix run-,$(MODULES))
	@echo "All host tests passed"

list:
	@echo "Discovered test modules: $(MODULES)"

run-%: $(BUILD_DIR)/test_%
	@echo "Running $*"
	@$(BUILD_DIR)/test_$*

test-file:
	@if [ -z "$(FILE)" ]; then \
		echo "FILE is required, e.g. make -f tests/HostTests.make test-file FILE=balance_manager"; \
		exit 2; \
	fi
	@if [ ! -f "$(TEST_ROOT)/test_$(FILE).c" ]; then \
		echo "Missing test file: $(TEST_ROOT)/test_$(FILE).c"; \
		exit 2; \
	fi
	@if [ ! -f "$(CORE_SRC)/$(FILE).c" ]; then \
		echo "Missing source file: $(CORE_SRC)/$(FILE).c"; \
		exit 2; \
	fi
	@$(MAKE) --no-print-directory -f $(firstword $(MAKEFILE_LIST)) run-$(FILE)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/test_%: $(TEST_ROOT)/test_%.c $(CORE_SRC)/%.c $(UNITY_SRC) $(SUPPORT_SRCS) | $(BUILD_DIR)
	$(CC) $(COMMON_CFLAGS) $(COMMON_INCLUDES) \
		$(UNITY_SRC) $(CORE_SRC)/$*.c $(TEST_ROOT)/test_$*.c $(SUPPORT_SRCS) \
		-o $@ $(COMMON_LDFLAGS)

clean:
	@rm -rf $(BUILD_DIR)
