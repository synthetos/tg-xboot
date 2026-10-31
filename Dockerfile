# Reproducible build environment for the TinyG bootloader.
# Release binaries are built with this image (see .github/workflows).
#
#   docker build -t tg-xboot-build .
#   docker run --rm -v "$PWD":/src -w /src tg-xboot-build make
#
# Debian trixie: avr-gcc 14.2, avr-libc 2.2.1
FROM debian:trixie-20261005-slim

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      gcc-avr binutils-avr avr-libc make \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src
CMD ["make"]
