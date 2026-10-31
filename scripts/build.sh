#!/bin/sh
# Build the TinyG bootloader on macOS or Linux.
#
#   scripts/build.sh                 native build if avr-gcc is installed, else Docker
#   scripts/build.sh --docker [...]  always build in the pinned Docker toolchain
#   scripts/build.sh --native [...]  always build with the local toolchain
#
# Extra arguments are passed to make, e.g. `scripts/build.sh clean` or
# `scripts/build.sh CONF=conf/x192a3.mk`.
set -eu

cd "$(dirname "$0")/.."

IMAGE=tg-xboot-build
mode=auto
case "${1:-}" in
    --docker) mode=docker; shift ;;
    --native) mode=native; shift ;;
esac

install_help() {
    cat >&2 <<'EOF'
No AVR toolchain (avr-gcc) or Docker found. Install one of:

  macOS (Homebrew):   brew tap osx-cross/avr && brew install avr-gcc avrdude
  Debian / Ubuntu:    sudo apt install gcc-avr binutils-avr avr-libc make avrdude
  Fedora:             sudo dnf install avr-gcc avr-binutils avr-libc make avrdude
  Arch:               sudo pacman -S avr-gcc avr-binutils avr-libc make avrdude

or install Docker and run: scripts/build.sh --docker
EOF
    exit 1
}

if [ "$mode" = auto ]; then
    if command -v avr-gcc >/dev/null 2>&1; then
        mode=native
    elif command -v docker >/dev/null 2>&1; then
        mode=docker
    else
        install_help
    fi
fi

case "$mode" in
native)
    command -v avr-gcc >/dev/null 2>&1 || install_help
    command -v make >/dev/null 2>&1 || { echo "make not found (macOS: xcode-select --install)" >&2; exit 1; }
    echo "Toolchain: $(avr-gcc --version | head -n1)"
    exec make "$@"
    ;;
docker)
    command -v docker >/dev/null 2>&1 || install_help
    docker build -q -t "$IMAGE" . >/dev/null
    echo "Toolchain: $(docker run --rm "$IMAGE" avr-gcc --version | head -n1) (docker)"
    exec docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/src -w /src "$IMAGE" make "$@"
    ;;
esac
