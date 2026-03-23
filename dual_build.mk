# Dual-bank application build wrapper for STM32Make.make
# Generates both bank artifacts in one command.

BASE_TARGET ?= BMS-Firmware-RTOS
INNER_MAKE ?= STM32Make.make
BUILD_ROOT ?= build
STAGE_DIRECTORY ?= $(BUILD_ROOT)/debug

BANK_A_BUILD_DIRECTORY := $(BUILD_ROOT)/bank_a
BANK_B_BUILD_DIRECTORY := $(BUILD_ROOT)/bank_b

COMMON_DEFS = -DSTM32L432xx -DSTM32_THREAD_SAFE_STRATEGY=4 -DUSE_HAL_DRIVER

.PHONY: all bank_a bank_b clean

all: bank_a bank_b

bank_a:
	powershell -NoProfile -Command "New-Item -ItemType Directory -Force '$(STAGE_DIRECTORY)' | Out-Null"
	$(MAKE) -f $(INNER_MAKE) \
		BUILD_DIRECTORY=$(BANK_A_BUILD_DIRECTORY) \
		TARGET=$(BASE_TARGET)_a \
		LDSCRIPT=STM32L432XX_APP_BANK_A.ld \
		C_DEFS="$(COMMON_DEFS) -DVECT_TAB_OFFSET=0x8000" \
		CXX_DEFS="$(COMMON_DEFS) -DVECT_TAB_OFFSET=0x8000"
	powershell -NoProfile -Command "Copy-Item '$(BANK_A_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_a.bin' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_a.bin' -Force; Copy-Item '$(BANK_A_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_a.elf' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_a.elf' -Force; Copy-Item '$(BANK_A_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_a.hex' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_a.hex' -Force; Copy-Item '$(BANK_A_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_a.lss' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_a.lss' -Force; Copy-Item '$(BANK_A_BUILD_DIRECTORY)/$(BASE_TARGET)_a.map' '$(BUILD_ROOT)/$(BASE_TARGET)_a.map' -Force"

bank_b:
	powershell -NoProfile -Command "New-Item -ItemType Directory -Force '$(STAGE_DIRECTORY)' | Out-Null"
	$(MAKE) -f $(INNER_MAKE) \
		BUILD_DIRECTORY=$(BANK_B_BUILD_DIRECTORY) \
		TARGET=$(BASE_TARGET)_b \
		LDSCRIPT=STM32L432XX_APP_BANK_B.ld \
		C_DEFS="$(COMMON_DEFS) -DVECT_TAB_OFFSET=0x22000" \
		CXX_DEFS="$(COMMON_DEFS) -DVECT_TAB_OFFSET=0x22000"
	powershell -NoProfile -Command "Copy-Item '$(BANK_B_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_b.bin' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_b.bin' -Force; Copy-Item '$(BANK_B_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_b.elf' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_b.elf' -Force; Copy-Item '$(BANK_B_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_b.hex' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_b.hex' -Force; Copy-Item '$(BANK_B_BUILD_DIRECTORY)/debug/$(BASE_TARGET)_b.lss' '$(STAGE_DIRECTORY)/$(BASE_TARGET)_b.lss' -Force; Copy-Item '$(BANK_B_BUILD_DIRECTORY)/$(BASE_TARGET)_b.map' '$(BUILD_ROOT)/$(BASE_TARGET)_b.map' -Force"

clean:
	$(MAKE) -f $(INNER_MAKE) BUILD_DIRECTORY=$(BANK_A_BUILD_DIRECTORY) clean
	$(MAKE) -f $(INNER_MAKE) BUILD_DIRECTORY=$(BANK_B_BUILD_DIRECTORY) clean
