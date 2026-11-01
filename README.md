<p align="center"><img src="docs/logo.svg" alt="tg-xboot" width="440"></p>

The bootloader for [TinyG](https://github.com/synthetos/TinyG) (ATxmega192A3).
It is a fork of Alex Forencich's [XBoot](https://github.com/alexforencich/xboot)
and speaks the AVR109 protocol over the TinyG USB serial port (USARTC0,
115200 baud), so firmware can be flashed with `avrdude -c avr109`.

## Building

You need `avr-gcc`, `avr-libc`, and GNU make. AVR Studio is not required.

```sh
scripts/build.sh            # checks for the toolchain, then runs make
make                        # same thing
```

Installing a toolchain:

| OS | Command |
|----|---------|
| macOS | `brew tap osx-cross/avr && brew install avr-gcc avrdude` |
| Debian/Ubuntu | `sudo apt install gcc-avr binutils-avr avr-libc make avrdude` |
| Fedora | `sudo dnf install avr-gcc avr-binutils avr-libc make avrdude` |

Output goes to `build/x192a3/`:

| File | Use |
|------|-----|
| `xboot.hex` | image at its real address (boot section, 0x30000) |
| `xboot-boot.hex` | same image relocated to 0, for avrdude's `boot` memory |
| `xboot.lss`, `xboot.map`, `xboot.sym` | listing, map, and symbols for inspection |

The build fails if the image doesn't fit in the 8 KB boot section.

## Configuration

All TinyG settings are in [`conf/x192a3.mk`](conf/x192a3.mk). The Makefile
turns them into `build/x192a3/config.h`. Option meanings are documented in
[docs/XBOOT.md](docs/XBOOT.md#3-configuring-xboot). To use a different config:
`make CONF=conf/other.mk`.

Bootloader behavior on TinyG:

- On reset, the LED (PF5) blinks for about 4.8 s. If the host sends ESC
  (0x1b) during that time, the board stays in the bootloader. Otherwise it
  starts the application.
- When TinyG reboots into the bootloader itself (`$boot=1` or `{"boot":1}`,
  detected through the software-reset flag), the window is 60 s.

## Flashing

**The bootloader**, through PDI (Atmel-ICE by default):

```sh
make program                              # chip erase + boot section + fuse2=0xBF
make program PROGRAMMER=avrispmkii        # other programmer
```

`make program` erases the whole chip, including the TinyG firmware and (unless
EESAVE is set) the EEPROM settings. To swap only the bootloader and keep
everything else, use `scripts/hwtest.sh`, which never chip-erases:

```sh
export ICE=<Atmel-ICE serial>                     # always name the ICE explicitly
scripts/hwtest.sh backup  backups/board1          # PDI read of everything, read twice and compared
scripts/hwtest.sh install backups/board1          # erase + write only the boot section, prove app/EEPROM unchanged
scripts/hwtest.sh restore backups/board1          # put the backed-up bootloader back
```

`scripts/avr109_probe.py` checks the bootloader protocol against such a backup
(identity, 2-byte addresses, extended addresses above 128 KB, EEPROM reads,
byte-write ACK) and measures the entry windows:

```sh
uv run --with pyserial scripts/avr109_probe.py probe        --port /dev/cu.usbserial-XXXX --ice $ICE --backup backups/board1
uv run --with pyserial scripts/avr109_probe.py window-reset --port /dev/cu.usbserial-XXXX --ice $ICE
uv run --with pyserial scripts/avr109_probe.py window-boot  --port /dev/cu.usbserial-XXXX
```

**TinyG firmware**, through the bootloader over USB serial:

```sh
avrdude -p x192a3 -c avr109 -P /dev/cu.usbserial-XXXX -b 115200 -U flash:w:tinyg.hex:i
```

Start avrdude within the entry window: reset the board, or send `$boot=1` first.

## Repository layout

| Path | Contents |
|------|----------|
| `src/` | bootloader source |
| `conf/` | build configurations |
| `app-api/` | client library that application firmware uses to call the bootloader API |
| `scripts/` | build and hardware-test helpers |
| `reference/` | the 2013 binary that has been deployed on TinyG boards |
| `docs/` | upstream XBoot documentation |

## CI and releases

GitHub Actions builds every push and pull request on Ubuntu 24.04 with its
AVR packages (`gcc-avr`, `avr-libc`) and uploads the hex files as artifacts,
along with `TOOLCHAIN.txt` recording the exact package versions. Pushing a
`v*` tag creates a **draft** release with the hex files and SHA256SUMS.
Publish the draft only after that exact artifact has been tested on a real
TinyG: CI's compiler differs from a local Homebrew build, so the binaries
differ too.

## License

GPLv2 (see [LICENSE](LICENSE)). The XBoot sources carry their original
license headers.
