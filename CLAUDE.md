# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The bootloader for TinyG (ATxmega192A3). It is a fork of Alex Forencich's XBoot v1.7 (https://github.com/alexforencich/xboot) and speaks AVR109 over USARTC0 at 115200 baud, so hosts flash firmware with `avrdude -c avr109`. These boards are in the field, so treat any behavior change as user-visible.

## Commands

```sh
make                         # build build/x192a3/{xboot.hex, xboot-boot.hex, .lss, .map, .sym}; fails if > 8 KB boot section
make CONF=conf/other.mk      # alternate config (output in build/other/)
scripts/build.sh [--docker]  # native if avr-gcc is installed, else the pinned Docker toolchain (avr-gcc 14)
make program                 # avrdude PDI (PROGRAMMER=atmelice_pdi): chip erase + boot + fuse2 -- wipes the TinyG app
scripts/hwtest.sh ...        # backup / install / restore / avr109 on real boards (ICE=<serial>, PORT=<tty>)
```

There are no unit tests, and simavr has no XMEGA core. Verify changes by:
1. Building with no warnings. CI also builds with Ubuntu's older avr-gcc under dash.
2. Reading `build/x192a3/xboot.lss` for the code you touched.
3. Comparing raw binaries (`avr-objcopy -I ihex -O binary`), not `.hex` text: record layout differs between objcopy versions.
4. Testing on hardware. Never push or tag until a build has passed hardware tests (user rule). Release tags create **draft** GitHub releases for that reason.

## Configuration flow

`conf/x192a3.mk` (make syntax) → the Makefile `include`s it → the `build/<conf>/config.h` rule writes every variable that was *defined in the conf file* (`$(origin) == file`) and whose name starts with `ATTACH_ ENABLE_ ENTER_ FIFO_ I2C_ LED_ LOCK_ UART_ USE_ WATCHDOG_`:
- `yes`/`y` → `#define NAME`
- `no`/`n`/empty → omitted
- anything else → `#define NAME value`

A new option needs one of those prefixes, or the C code never sees it. `src/xboot.h` then derives the rest: port/USART/ISR names through token pasting, UART BSEL/BSCALE from a table keyed on `F_CPU` and `UART_BAUD_RATE`, and entry-loop timing. Its built-in default block applies only without `USE_CONFIG_H`, which the Makefile always defines. A misspelled option name compiles silently as undefined.

Pitfalls:
- Don't put inline `# comments` after values in conf files: make keeps the trailing whitespace.
- The Makefile must stay compatible with GNU make 3.81 (macOS). No `$(file ...)`.

## Code structure

`src/xboot.c` `main()` runs straight through:
1. **Init**: TinyG clock (external 16 MHz crystal ×2 PLL, set with `_PROTECTED_WRITE`), vector table remapped to the boot section, LED and UART.
2. **Entry window**: polls the triggers every `ENTER_POLL_US` (a cycle-exact `_delay_us`, so the timing doesn't depend on the compiler) and toggles the LED every `ENTER_BLINK_MS`. TinyG requires ESC (`CMD_SYNC`) to enter. The software-reset flag (`$boot=1`) selects `ENTER_BLINK_COUNT_SWRESET`.
3. **Command loop**: AVR109 parser (opcodes in `protocol.h`). TinyG uses the interrupt-driven `get_char`/`send_char`, which have 2-byte RX buffers.
4. **Exit**: `install_firmware()` (applies an image staged through the API), peripheral de-init, vector table restored, `jmp 0`.

Other pieces:
- `flash.c`/`sp_driver.S`: Atmel NVM/SPM driver. `eeprom_driver.c`: XMEGA EEPROM. `i2c.c` and `fifo.c` compile to nothing unless `USE_I2C`/`USE_FIFO` is set.
- `api.c`: jump table placed in `.vectors` (magic `XBj`, version 1) at a fixed offset that application firmware calls. Don't reorder its entries. Any change there breaks the app ABI. `app-api/` is the application-side client and is not part of this build.
- The linker may reorder the unused `gs()` trampoline stubs after the jump table when object paths change. That is harmless; the table entries point directly at the functions.

## Compiler-sensitive history

The 2013 code had inline asm that called functions without declaring clobbered registers (`get_2bytes`, ext-address). It worked under GCC 4.x only because of how registers happened to be allocated, and it breaks under GCC 9+. It is now plain C. Don't add inline asm that calls C functions. When changing compilers, compare `get_char`/`get_2bytes`/the command loop in the `.lss` against `reference/xboot-tinyg-2013.hex` (the deployed baseline; see `reference/README.md`).
