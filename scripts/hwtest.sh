#!/bin/sh
# Hardware test helper: TinyG (ATxmega192A3) + Atmel-ICE (PDI) + USB serial.
#
# Environment:
#   ICE=<serial>     Atmel-ICE USB serial number (required: with several ICEs
#                    attached avrdude would otherwise pick one arbitrarily)
#   PORT=<device>    TinyG USB serial port, e.g. /dev/cu.usbserial-DN01ABCD
#   BOOT_HEX=<file>  bootloader to install (default build/x192a3/xboot-boot.hex)
#
# Commands:
#   backup  <dir>        Read app, boot, EEPROM, user signature, fuses, lock via PDI.
#   install <backupdir>  Erase and write ONLY the boot section with BOOT_HEX, then
#                        verify application + EEPROM are byte-identical to the backup.
#   restore <backupdir>  Same, writing back the backed-up boot section (returns the
#                        board to its previous bootloader).
#
# Never chip-erases: with EESAVE off that would wipe the EEPROM settings.
# Fuses and lock bits are never written.
#   reset                Reset the board through the ICE (starts the bootloader).
#   avr109  <image.hex>  Reset via ICE, then flash + verify the image over USB
#                        (avrdude -D; set ERASE=1 to let avrdude send the
#                        bootloader's chip erase 'e' first)
#                        serial through the bootloader (needs PORT).
set -eu

cd "$(dirname "$0")/.."

MCU=x192a3
BOOT_HEX=${BOOT_HEX:-build/x192a3/xboot-boot.hex}
[ -n "${ICE:-}" ] || { echo "set ICE=<Atmel-ICE serial number>" >&2; exit 2; }
ICE_PORT=usb:$ICE

pdi() { avrdude -p "$MCU" -c atmelice_pdi -P "$ICE_PORT" "$@"; }

need_dir() {
    [ -n "${1:-}" ] || { echo "usage: $0 $cmd <dir>" >&2; exit 2; }
}

check_backup() {
    [ -e "$1/VERIFIED" ] || { echo "backup $1 was not verified (make it with: $0 backup <dir>)" >&2; exit 1; }
    (cd "$1" && shasum -a 256 -c --quiet SHA256SUMS) || { echo "backup $1 fails its checksums" >&2; exit 1; }
    for f in application.hex boot.hex eeprom.hex fuse2.txt; do
        [ -s "$1/$f" ] || { echo "backup $1 is incomplete (missing $f)" >&2; exit 1; }
    done
}

# Replace only the boot section, then prove nothing else changed.
swap_boot() {
    backup=$1 image=$2
    pdi -D -T "erase boot" -U boot:w:"$image":i
    t=$(mktemp -d)
    pdi -U application:r:"$t/application.hex":i -U eeprom:r:"$t/eeprom.hex":i
    if cmp -s "$t/application.hex" "$backup/application.hex" &&
       cmp -s "$t/eeprom.hex" "$backup/eeprom.hex"; then
        echo "OK: boot section replaced; application and EEPROM unchanged"
        rm -rf "$t"
    else
        echo "ERROR: application or EEPROM differs from backup! Reads kept in $t" >&2
        exit 1
    fi
}

cmd=${1:-}; shift || true
case "$cmd" in
backup)
    need_dir "${1:-}"
    dir=$1
    [ ! -e "$dir" ] || { echo "$dir already exists; refusing to overwrite a backup" >&2; exit 1; }
    tmp="$dir.partial"
    rm -rf "$tmp"; mkdir -p "$tmp"
    pdi -U lock:r:"$tmp/lock.txt":h
    if ! grep -qi '^0xff$' "$tmp/lock.txt"; then
        echo "ERROR: lock bits are $(cat "$tmp/lock.txt") (not 0xff); flash reads may be blank." >&2
        echo "Refusing to make a backup that install would trust. Partial data in $tmp" >&2
        exit 1
    fi
    pdi -U application:r:"$tmp/application.hex":i \
        -U boot:r:"$tmp/boot.hex":i \
        -U eeprom:r:"$tmp/eeprom.hex":i \
        -U usersig:r:"$tmp/usersig.hex":i \
        -U fuse0:r:"$tmp/fuse0.txt":h -U fuse1:r:"$tmp/fuse1.txt":h \
        -U fuse2:r:"$tmp/fuse2.txt":h -U fuse4:r:"$tmp/fuse4.txt":h \
        -U fuse5:r:"$tmp/fuse5.txt":h
    # Second read to make sure the backup is stable before anything trusts it
    pdi -U application:r:"$tmp/application.verify.hex":i -U eeprom:r:"$tmp/eeprom.verify.hex":i
    cmp -s "$tmp/application.hex" "$tmp/application.verify.hex" &&
        cmp -s "$tmp/eeprom.hex" "$tmp/eeprom.verify.hex" ||
        { echo "ERROR: second read differs from first; backup left in $tmp" >&2; exit 1; }
    rm -f "$tmp"/*.verify.hex
    (cd "$tmp" && shasum -a 256 ./*.hex > SHA256SUMS && touch VERIFIED)
    mv "$tmp" "$dir"
    echo "Backup written and verified: $dir"
    for f in fuse0 fuse1 fuse2 fuse4 fuse5 lock; do printf '  %-6s %s\n' "$f" "$(cat "$dir/$f.txt")"; done
    # fuse2 bit 6 (BOOTRST) must be 0 so reset starts the bootloader
    [ $(( $(cat "$dir/fuse2.txt") & 0x40 )) -eq 0 ] || echo "WARNING: fuse2 BOOTRST is not set to the boot section"
    ;;
install)
    need_dir "${1:-}"; check_backup "$1"
    [ -s "$BOOT_HEX" ] || { echo "$BOOT_HEX not found; run make first" >&2; exit 1; }
    swap_boot "$1" "$BOOT_HEX"
    ;;
restore)
    need_dir "${1:-}"; check_backup "$1"
    swap_boot "$1" "$1/boot.hex"
    ;;
reset)
    pdi -U signature:r:-:h >/dev/null
    echo "Board reset; bootloader window is open"
    ;;
avr109)
    [ -n "${1:-}" ] && [ -s "$1" ] || { echo "usage: $0 avr109 <image.hex>" >&2; exit 2; }
    [ -n "${PORT:-}" ] || { echo "set PORT to the TinyG serial device" >&2; exit 2; }
    pdi -U signature:r:-:h >/dev/null
    if [ "${ERASE:-0}" = 1 ]; then noerase=""; else noerase="-D"; fi
    avrdude -p "$MCU" -c avr109 -P "$PORT" -b 115200 $noerase -U flash:w:"$1":i
    ;;
*)
    sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
