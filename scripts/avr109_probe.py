#!/usr/bin/env python3
"""AVR109 protocol probe for the TinyG XBoot bootloader (hardware test).

Resets the board through the Atmel-ICE, enters the bootloader over USB serial
and checks individual protocol commands against a PDI backup made with
`scripts/hwtest.sh backup`. Nothing is written except one EEPROM byte, which
is written back with the value it already has.

  uv run --with pyserial scripts/avr109_probe.py probe        --port P --ice SN --backup DIR
  uv run --with pyserial scripts/avr109_probe.py window-reset --port P --ice SN
  uv run --with pyserial scripts/avr109_probe.py window-boot  --port P

window-reset: time from ICE reset to the first byte from the application.
window-boot:  same, after sending {"boot":1} to the running application.
"""
import argparse
import subprocess
import sys
import time

import serial

PAGE = 512
EEPROM_SIZE = 0x800   # ATxmega192A3: 2 KB (addresses above mirror)


def load_ihex(path, size):
    """Intel HEX -> bytearray of `size`, unfilled bytes 0xFF."""
    mem = bytearray(b"\xff" * size)
    base = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            raw = bytes.fromhex(line[1:])
            n, addr, rtype, data = raw[0], (raw[1] << 8) | raw[2], raw[3], raw[4:4 + raw[0]]
            if rtype == 0:
                a = base + addr
                mem[a:a + n] = data
            elif rtype == 2:
                base = ((data[0] << 8) | data[1]) << 4
            elif rtype == 4:
                base = ((data[0] << 8) | data[1]) << 16
    return mem


def ice_reset(ice):
    subprocess.run(
        ["avrdude", "-q", "-q", "-c", "atmelice_pdi", "-P", f"usb:{ice}", "-p", "x192a3",
         "-U", "signature:r:-:h"],
        check=True, stdout=subprocess.DEVNULL)


class Boot:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=1.0)

    def cmd(self, out, nreply, timeout=1.0):
        self.s.timeout = timeout
        self.s.write(out)
        return self.s.read(nreply)

    def enter(self, deadline=4.0):
        """Send ESC + 'S' until the bootloader identifies itself."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < deadline:
            self.s.reset_input_buffer()
            self.s.write(b"\x1b")
            if self.cmd(b"S", 7, timeout=0.15) == b"XBoot++":
                return time.monotonic() - t0
        return None


results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{(': ' + detail) if detail else ''}")


def probe(a):
    app = load_ihex(f"{a.backup}/application.hex", 0x30000)
    ee = load_ihex(f"{a.backup}/eeprom.hex", EEPROM_SIZE)
    b = Boot(a.port)
    b.s.reset_input_buffer()
    ice_reset(a.ice)
    t = b.enter()
    if t is None:
        print("Bootloader did not answer (no 'XBoot++' within 4 s of reset)")
        sys.exit(1)
    print(f"Entered bootloader {t:.2f} s after ICE reset")

    check("program id 'S'", True, "XBoot++")
    v = b.cmd(b"V", 2)
    check("version 'V'", v == b"17", repr(v))
    check("programmer type 'p'", b.cmd(b"p", 1) == b"S")
    check("autoincrement 'a'", b.cmd(b"a", 1) == b"Y")
    r = b.cmd(b"b", 3)
    check("block support 'b' = Y, 0x0200", r == b"Y\x02\x00", r.hex())
    r = b.cmd(b"s", 3)
    check("signature 's' = 44 97 1e", r == bytes([0x44, 0x97, 0x1E]), r.hex())

    def read_flash(set_addr, byte_addr, label):
        ack = b.cmd(set_addr, 1)
        data = b.cmd(b"g\x02\x00F", PAGE, timeout=2.0)
        want = bytes(app[byte_addr:byte_addr + PAGE])
        check(label, ack == b"\r" and data == want,
              f"ack={ack!r} got {len(data)} B, {'match' if data == want else 'MISMATCH'}"
              + ("" if want != b"\xff" * PAGE else " (region is blank in backup)"))

    # 'A' with a nonzero high address byte and a 0x0200 block size both go
    # through get_2bytes(), the routine that was miscompiled by modern GCC.
    read_flash(b"A\x00\x00", 0x00000, "flash read @0x00000 ('A' + 'g')")
    read_flash(b"A\x81\x00", 0x10200, "flash read @0x10200 ('A' 0x8100, high byte set)")
    # 'H' = extended address (needed above 128 KB)
    read_flash(b"H\x01\x00\x00", 0x20000, "flash read @0x20000 ('H' ext address)")
    read_flash(b"H\x01\x6f\x00", 0x2DE00, "flash read @0x2DE00 ('H' ext address)")

    ok = b.cmd(b"A\x00\x00", 1) == b"\r"
    got = b""
    for _ in range(EEPROM_SIZE // PAGE):
        got += b.cmd(b"g\x02\x00E", PAGE, timeout=2.0)
    check("EEPROM block read 2 KB ('g' E)", ok and got == bytes(ee),
          "match" if got == bytes(ee) else f"MISMATCH ({len(got)} B)")

    # Byte write: rewrite the last EEPROM byte with its current value.
    addr = EEPROM_SIZE - 1
    val = ee[addr]
    b.cmd(b"A" + bytes([addr >> 8, addr & 0xFF]), 1)
    r = b.cmd(b"D" + bytes([val]), 1, timeout=1.0)
    check("EEPROM byte write 'D' sends ACK", r == b"\r",
          repr(r) if r else "no reply (2013 bug: missing ACK)")
    if not r:
        b.s.reset_input_buffer()
    b.cmd(b"A" + bytes([addr >> 8, addr & 0xFF]), 1)
    r = b.cmd(b"d", 1)
    check("EEPROM byte read back 'd'", r == bytes([val]), f"{r.hex()} want {val:02x}")

    for c, name in ((b"r", "lock"), (b"F", "fuse 'F'"), (b"N", "fuse 'N'"), (b"Q", "fuse 'Q'")):
        r = b.cmd(c, 1)
        print(f"  [info] {name}: {r.hex() if r else 'no reply'}")

    b.s.reset_input_buffer()
    r = b.cmd(b"E", 1)
    check("exit 'E' acks", r == b"\r", repr(r))
    t0 = time.monotonic()
    b.s.timeout = 10
    first = b.s.read(1)
    check("application starts after exit", bool(first),
          f"first byte after {time.monotonic() - t0:.2f} s" if first else "no output in 10 s")
    print(f"\n{sum(results)}/{len(results)} checks passed")
    sys.exit(0 if all(results) else 1)


def wait_app_output(s, limit):
    t0 = time.monotonic()
    s.timeout = limit
    first = s.read(1)
    return (time.monotonic() - t0) if first else None


def window_reset(a):
    s = serial.Serial(a.port, 115200)
    s.reset_input_buffer()
    ice_reset(a.ice)
    t = wait_app_output(s, 30)
    print(f"ICE reset -> first application byte: {t:.2f} s" if t else "no application output in 30 s")


def window_boot(a):
    s = serial.Serial(a.port, 115200, timeout=0.5)
    s.reset_input_buffer()
    s.write(b'{"boot":1}\n')
    s.read(4096)                 # discard the reply to the command itself
    s.reset_input_buffer()
    t = wait_app_output(s, 120)
    print(f'{{"boot":1}} -> first application byte: {t + 0.5:.1f} s' if t else "no application output in 120 s")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["probe", "window-reset", "window-boot"])
    p.add_argument("--port", required=True)
    p.add_argument("--ice")
    p.add_argument("--backup")
    a = p.parse_args()
    if a.mode in ("probe", "window-reset") and not a.ice:
        p.error("--ice SERIAL is required")
    if a.mode == "probe" and not a.backup:
        p.error("--backup DIR is required")
    {"probe": probe, "window-reset": window_reset, "window-boot": window_boot}[a.mode](a)
