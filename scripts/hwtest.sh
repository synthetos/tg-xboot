#!/bin/sh
# Hardware test helper: TinyG (ATxmega192A3) + Atmel-ICE (PDI) + USB serial.
#
# Environment:
#   ICE=<serial>     Atmel-ICE USB serial number (required if more than one is
#                    attached; see `avrdude -c atmelice_pdi -P usb:xx -p x192a3`)
#   PORT=<device>    TinyG USB serial port, e.g. /dev/cu.usbserial-DN01ABCD
#   BOOT_HEX=<file>  bootloader to install (default build/x192a3/xboot-boot.hex)
#
# Commands:
#   backup  <dir>        Read app, boot, EEPROM, user signature, fuses, lock via PDI.
#   install <backupdir>  Chip erase, write BOOT_HEX, restore app + EEPROM from the
#                        backup, verify. Fuses are not touched.
#   restore <backupdir>  Chip erase and write back boot + app + EEPROM exactly as
#                        backed up (returns the board to its previous bootloader).
#   reset                Reset the board through the ICE (starts the bootloader).
#   avr109  <image.hex>  Reset via ICE, then flash + verify the image over USB
#                        serial through the bootloader (needs PORT).
set -eu

cd "$(dirname "$0")/.."

MCU=x192a3
BOOT_HEX=${BOOT_HEX:-build/x192a3/xboot-boot.hex}
ICE_PORT=usb${ICE:+:$ICE}

pdi() { avrdude -p "$MCU" -c atmelice_pdi -P "$ICE_PORT" "$@"; }

need_dir() {
    [ -n "${1:-}" ] || { echo "usage: $0 $cmd <dir>" >&2; exit 2; }
}

check_backup() {
    for f in application.hex boot.hex eeprom.hex fuse2.txt; do
        [ -s "$1/$f" ] || { echo "backup $1 is incomplete (missing $f)" >&2; exit 1; }
    done
}

cmd=${1:-}; shift || true
case "$cmd" in
backup)
    need_dir "${1:-}"
    dir=$1
    [ ! -e "$dir" ] || { echo "$dir already exists; refusing to overwrite a backup" >&2; exit 1; }
    mkdir -p "$dir"
    pdi -U application:r:"$dir/application.hex":i \
        -U boot:r:"$dir/boot.hex":i \
        -U eeprom:r:"$dir/eeprom.hex":i \
        -U usersig:r:"$dir/usersig.hex":i \
        -U fuse0:r:"$dir/fuse0.txt":h -U fuse1:r:"$dir/fuse1.txt":h \
        -U fuse2:r:"$dir/fuse2.txt":h -U fuse4:r:"$dir/fuse4.txt":h \
        -U fuse5:r:"$dir/fuse5.txt":h -U lock:r:"$dir/lock.txt":h
    (cd "$dir" && shasum -a 256 ./*.hex > SHA256SUMS)
    echo "Backup written to $dir"
    grep -i '0xbf' "$dir/fuse2.txt" >/dev/null || echo "WARNING: fuse2 is $(cat "$dir/fuse2.txt"), expected 0xbf (reset into boot section)"
    ;;
install)
    need_dir "${1:-}"; check_backup "$1"
    [ -s "$BOOT_HEX" ] || { echo "$BOOT_HEX not found; run make first" >&2; exit 1; }
    pdi -e -U boot:w:"$BOOT_HEX":i \
        -U application:w:"$1/application.hex":i \
        -U eeprom:w:"$1/eeprom.hex":i
    ;;
restore)
    need_dir "${1:-}"; check_backup "$1"
    pdi -e -U boot:w:"$1/boot.hex":i \
        -U application:w:"$1/application.hex":i \
        -U eeprom:w:"$1/eeprom.hex":i
    ;;
reset)
    pdi -U signature:r:-:h >/dev/null
    echo "Board reset; bootloader window is open"
    ;;
avr109)
    [ -n "${1:-}" ] && [ -s "$1" ] || { echo "usage: $0 avr109 <image.hex>" >&2; exit 2; }
    [ -n "${PORT:-}" ] || { echo "set PORT to the TinyG serial device" >&2; exit 2; }
    pdi -U signature:r:-:h >/dev/null
    avrdude -p "$MCU" -c avr109 -P "$PORT" -b 115200 -U flash:w:"$1":i
    ;;
*)
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
