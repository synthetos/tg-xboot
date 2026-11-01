# XBoot application API (client side)

`xbootapi.c` / `xbootapi.h` belong in **application firmware**, not the
bootloader. They call the jump table that the bootloader places in its boot
section (`src/api.c`, magic `"XBj"`, API version 1). With it, the running
application can:

- query the bootloader version
- erase and write application flash pages through the boot-section SPM wrapper
- stage a new firmware image in the temporary area; the bootloader installs it
  on its next exit (`install_firmware()` in `src/xboot.c`)

See section 4 of [docs/XBOOT.md](../docs/XBOOT.md) for the API reference.
