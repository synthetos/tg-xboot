#
# TinyG (ATxmega192A3) bootloader configuration
#
# Every variable with one of the prefixes listed in CONFIG_PREFIXES (see
# Makefile) is written to build/<conf>/config.h:
#   yes / y      -> #define NAME
#   no / n / ""  -> (omitted)
#   other        -> #define NAME value
# Option meanings are documented in docs/XBOOT.md, section 3.
#

MCU   = atxmega192a3
# 16 MHz crystal x2 PLL (see clock setup in src/xboot.c)
F_CPU = 32000000

# Programming: fuse2 = 0xBF puts the reset vector in the boot section
FUSES = -U fuse2:w:0xBF:m

# --- Entry -------------------------------------------------------------
USE_ENTER_DELAY      = yes
USE_ENTER_PIN        = no
USE_ENTER_UART       = yes
USE_ENTER_I2C        = no
USE_ENTER_FIFO       = no
# host must send ESC (0x1b) to stay in the bootloader
ENTER_UART_NEED_SYNC = yes

# Normal entry window: ENTER_BLINK_COUNT blinks, LED toggles every ENTER_BLINK_MS
# (12 x 2 x 200 ms = 4.8 s; the 2013 build was ~4.5 s).
ENTER_BLINK_COUNT = 12
ENTER_BLINK_MS    = 200
# Window after the app requests the bootloader ($boot=1 / {"boot":1}),
# detected via the software reset flag: 150 x 2 x 200 ms = 60 s.
ENTER_BLINK_COUNT_SWRESET = 150

# --- Exit --------------------------------------------------------------
LOCK_SPM_ON_EXIT = no

# --- Communication -----------------------------------------------------
USE_LED  = yes
USE_UART = yes
USE_UART_EN_PIN = no
USE_I2C  = no
USE_FIFO = no

LED_PORT_NAME = F
LED_PIN       = 5
LED_INV       = 0

UART_BAUD_RATE = 115200
# USARTC0 -> TinyG USB serial
UART_PORT_NAME = C
UART_NUMBER    = 0
UART_RX_PUEN   = yes

# --- General -----------------------------------------------------------
USE_AVR1008_EEPROM = no
USE_INTERRUPTS     = yes
USE_WATCHDOG       = yes
WATCHDOG_TIMEOUT   = WDT_PER_1KCLK_gc

# --- Features ----------------------------------------------------------
ENABLE_BLOCK_SUPPORT       = yes
ENABLE_FLASH_BYTE_SUPPORT  = yes
ENABLE_EEPROM_BYTE_SUPPORT = yes
# Lock-bit commands off: flashing never needs them, and a stray 'l' in text
# sent to a board left in command mode could lock the chip (issue #3).
ENABLE_LOCK_BITS           = no
# Fuse bits are read-only from the bootloader ('F' 'N' 'Q').
ENABLE_FUSE_BITS           = yes
ENABLE_FLASH_ERASE_WRITE   = yes
ENABLE_CRC_SUPPORT         = yes
# Chip erase ('e') erases the application section only; the EEPROM holds the
# TinyG settings and is never erased by the bootloader (issue #3).
ENABLE_CHIP_ERASE_EEPROM   = no
# Erase/write commands are refused until the host sends 'T' <device code>
# followed directly by 'P' (the end of avrdude's handshake). Any unknown
# byte locks them again, so text sent to a board left in command mode after
# an interrupted flash cannot write anything (issue #3).
USE_WRITE_ARMING           = yes
# After this many consecutive unknown command bytes (e.g. JSON or G-code sent
# to a board left in command mode), reset back to the entry window instead of
# executing the text as commands. avrdude's only unsupported command in a
# normal session is 'v', always followed by a supported one.
USE_BAD_COMMAND_LIMIT      = 2

# --- Application API (see app-api/) ------------------------------------
ENABLE_API                 = yes
USE_API_VERSION            = 1
ENABLE_API_LOW_LEVEL_FLASH = yes
ENABLE_API_SPM_WRAPPER     = yes
ENABLE_API_FIRMWARE_UPDATE = yes

# --- Code protection ---------------------------------------------------
ENABLE_CODE_PROTECTION       = no
ENABLE_EEPROM_PROTECTION     = no
ENABLE_BOOTLOADER_PROTECTION = no
