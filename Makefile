# TinyG XBoot bootloader
#
#   make                      build (default config: conf/x192a3.mk)
#   make CONF=conf/other.mk   build another config
#   make program              flash bootloader + fuses with avrdude (chip erase!)
#   make clean                remove build/
#
# Outputs go to build/<conf-name>/. Works with GNU make 3.81+ (macOS stock).

CONF ?= conf/x192a3.mk
ifeq ($(wildcard $(CONF)),)
  $(error Config file "$(CONF)" not found)
endif
include $(CONF)

CONF_NAME := $(basename $(notdir $(CONF)))
BUILD     ?= build/$(CONF_NAME)
TARGET    := $(BUILD)/xboot

CROSS   ?= avr-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump
SIZE    := $(CROSS)size
NM      := $(CROSS)nm
AVRDUDE ?= avrdude

PROGRAMMER ?= atmelice_pdi
PORT       ?= usb

# Boot section start / size (bytes) for supported parts
BOOT_atxmega64a3   := 0x010000 4096
BOOT_atxmega64a3u  := 0x010000 4096
BOOT_atxmega128a3  := 0x020000 8192
BOOT_atxmega128a3u := 0x020000 8192
BOOT_atxmega192a3  := 0x030000 8192
BOOT_atxmega192a3u := 0x030000 8192
BOOT_atxmega256a3  := 0x040000 8192
BOOT_atxmega256a3u := 0x040000 8192
ifeq ($(BOOT_$(MCU)),)
  $(error MCU '$(MCU)' has no boot section entry in the Makefile)
endif
BOOT_SECTION_START := $(word 1,$(BOOT_$(MCU)))
BOOT_SECTION_SIZE  := $(word 2,$(BOOT_$(MCU)))

# Link order matters for the binary layout; keep it stable.
SRC  := src/xboot.c src/flash.c src/eeprom_driver.c src/uart.c src/i2c.c \
        src/fifo.c src/watchdog.c src/api.c
ASRC := src/sp_driver.S
OBJ  := $(patsubst src/%.c,$(BUILD)/%.o,$(SRC)) $(patsubst src/%.S,$(BUILD)/%.o,$(ASRC))

COMMON  := -mmcu=$(MCU) -I$(BUILD) -Isrc -DF_CPU=$(strip $(F_CPU))L -DUSE_CONFIG_H
CFLAGS  := $(COMMON) -g -Os -std=gnu99 \
           -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
           -ffunction-sections -fdata-sections -fno-jump-tables \
           -Wall -Wstrict-prototypes -MMD -MP
ASFLAGS := $(COMMON) -x assembler-with-cpp -MMD -MP
LDFLAGS := -mmcu=$(MCU) -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections \
           -Wl,--section-start=.text=$(BOOT_SECTION_START)

# ---------------------------------------------------------------------------
# config.h generation: every make variable defined in $(CONF) (or on the
# make command line) whose name
# starts with one of these prefixes becomes a #define.
CONFIG_PREFIXES := ATTACH_ ENABLE_ ENTER_ FIFO_ I2C_ LED_ LOCK_ UART_ USE_ WATCHDOG_
CONFIG_VARS = $(sort $(foreach v,$(filter $(addsuffix %,$(CONFIG_PREFIXES)),$(.VARIABLES)),\
                $(if $(filter file command line,$(origin $(v))),$(v))))
HASH := \#
config_line = $(if $(filter yes y,$(strip $($1))),'$(HASH)define $1',\
              $(if $(filter-out no n,$(strip $($1))),'$(HASH)define $1 $(strip $($1))'))

# ---------------------------------------------------------------------------
.PHONY: all clean size check-size program help FORCE
FORCE:
.DEFAULT_GOAL := all

all: $(TARGET).hex $(TARGET)-boot.hex $(TARGET).lss $(TARGET).sym check-size

$(BUILD):
	@mkdir -p $@

# Re-evaluated on every make (picks up conf and command-line changes) but
# rewritten only when its contents change, so objects rebuild only then.
$(BUILD)/config.h: FORCE | $(BUILD)
	@printf '%s\n' '// Generated from $(CONF) - do not edit' \
		'// MCU: $(MCU)  F_CPU: $(strip $(F_CPU))' \
		$(foreach v,$(CONFIG_VARS),$(call config_line,$(v))) > $@.tmp
	@if cmp -s $@.tmp $@; then rm -f $@.tmp; else mv $@.tmp $@; echo "Generated $@"; fi

$(OBJ): $(BUILD)/config.h

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: src/%.S | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Same image relocated to address 0, for avrdude's "boot" memory
$(TARGET)-boot.hex: $(TARGET).hex
	$(OBJCOPY) -I ihex -O ihex --change-addresses -$(BOOT_SECTION_START) $< $@

$(TARGET).lss: $(TARGET).elf
	$(OBJDUMP) -h -S $< > $@

$(TARGET).sym: $(TARGET).elf
	$(NM) -n $< > $@

size: $(TARGET).elf
	$(SIZE) -A $<

check-size: $(TARGET).elf
	@used=$$($(SIZE) -A $< | awk '$$1==".text" || $$1==".data" {s+=$$2} END {print s}'); \
	echo "Boot image: $$used / $(BOOT_SECTION_SIZE) bytes"; \
	if [ -z "$$used" ]; then echo "ERROR: could not determine image size" >&2; exit 1; fi; \
	if [ "$$used" -gt "$(BOOT_SECTION_SIZE)" ]; then \
		echo "ERROR: bootloader does not fit in the boot section" >&2; exit 1; fi

# Chip erase (wipes the application and, unless EESAVE is set, EEPROM),
# write the boot section, set fuses. Reflash TinyG afterwards via avr109.
program: $(TARGET)-boot.hex
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -e \
		-U boot:w:$(TARGET)-boot.hex:i $(FUSES)

clean:
	rm -rf build

help:
	@sed -n '2,8p' Makefile | sed 's/^# \{0,1\}//'

-include $(OBJ:.o=.d)
