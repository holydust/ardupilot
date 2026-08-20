#!/usr/bin/env bash
#
# CORVON G1 / G2 onboard I2C diagnosis over SWD, with the CPU halted.
#
# For the case where `test` reports `mag: FAIL n=0`. Both checks run against
# the live board through the same flying-lead probe used for flashing, so a
# board can be diagnosed without unsoldering anything or touching a scope.
#
#   scan    bit-bangs a full 0x08-0x77 address sweep on I2C1 and lists what
#           acknowledges. No ACK anywhere means the device is not reachable;
#           an ACK at an unexpected address means the address straps are wrong.
#
#   pullup  answers the question the scan cannot: is the compass-side 4.7K
#           actually reachable from the MCU? Reading SCL/SDA as high proves
#           nothing on its own - an open analog switch leaves the pins
#           floating and its ESD diode to V+ leaks them high. This sinks each
#           line and releases it; only a real pull-up snaps it back. It repeats
#           the measurement with the RS2058 commanded both ways, so the FC-side
#           pass doubles as the control that proves the switch is switching.
#
# Found on G2 2026-08-20: scan was empty and reflowing the IST8310 changed
# nothing, because the fault was not at the device - `pullup` showed SDA
# recovering and SCL not, which localised it to the SCL leg alone.
#
# Leaves the pins the way ChibiOS had them and resets the board on exit.

set -euo pipefail

usage() {
    cat >&2 <<EOF
usage: $(basename "$0") [scan|pullup|both] [--speed <kHz>]

  scan    bit-banged 0x08-0x77 address sweep on I2C1  (default: both)
  pullup  drive-low/release test on SCL and SDA, mux commanded both ways
EOF
    exit 2
}

WHAT=both
SPEED=480
while [ $# -gt 0 ]; do
    case "$1" in
        scan|pullup|both) WHAT="$1"; shift ;;
        --speed) SPEED="$2"; shift 2 ;;
        *) usage ;;
    esac
done

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# hla, same as the flash script: it tolerates a part sitting in lockup, and
# these checks are most often wanted on a board that is not behaving
run() {
    openocd -f interface/stlink.cfg \
            -c "transport select hla_swd" -c "adapter speed ${SPEED}" \
            -f target/stm32f4x.cfg -c init \
            -f "$DIR/$1" -c "reset run" -c shutdown
}

# plain ifs, not an && chain: under `set -e` a false test at the end of an
# AND-OR list aborts the script, so `pullup` alone would never reach its run
if [ "$WHAT" = scan ] || [ "$WHAT" = both ]; then
    run i2c_scan.cfg
fi
if [ "$WHAT" = pullup ] || [ "$WHAT" = both ]; then
    run i2c_pullup.cfg
fi
