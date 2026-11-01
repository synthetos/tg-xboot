#!/bin/sh
# Build the TinyG bootloader on macOS or Linux with the local AVR toolchain.
#
#   scripts/build.sh            build (default config)
#   scripts/build.sh clean      any arguments are passed to make
#
# Release binaries are built by GitHub Actions (.github/workflows/build.yml).
set -eu

cd "$(dirname "$0")/.."

if ! command -v avr-gcc >/dev/null 2>&1; then
    cat >&2 <<'EOF'
avr-gcc not found. Install an AVR toolchain:

  macOS (Homebrew):   brew tap osx-cross/avr && brew install avr-gcc avrdude
  Debian / Ubuntu:    sudo apt install gcc-avr binutils-avr avr-libc make avrdude
  Fedora:             sudo dnf install avr-gcc avr-binutils avr-libc make avrdude
  Arch:               sudo pacman -S avr-gcc avr-binutils avr-libc make avrdude
EOF
    exit 1
fi
command -v make >/dev/null 2>&1 || { echo "make not found (macOS: xcode-select --install)" >&2; exit 1; }

echo "Toolchain: $(avr-gcc --version | head -n1)"
exec make "$@"
