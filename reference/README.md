# Reference binary

`xboot-tinyg-2013.hex` is the TinyG bootloader built in 2013 (repo commit
`cec165a`) and deployed on TinyG boards since then. It is kept as a known-good
baseline for comparing behavior and disassembly. It is **not** built from the
current source.

```
sha256  2378bf2a1c7a53ad16fe9d0ce98a3124e36f8cbe789a852a67917253805ee156
```

It is linked at the boot section (0x30000). To flash it with avrdude's `boot`
memory, relocate it to address 0 first:

```sh
avr-objcopy -I ihex -O ihex --change-addresses -0x30000 xboot-tinyg-2013.hex xboot-tinyg-2013-boot.hex
```

Its known issues, all fixed in the current source: a corrupted `get_2bytes`
when rebuilt with modern GCC (it works only because GCC 4.x allocated r19),
an 8-bit overflow that cuts the `$boot=1` window to about 42 s, a chip-erase
wait that tests the wrong NVM status bits, no ACK for single EEPROM byte
writes, and a 2.1% baud error.
