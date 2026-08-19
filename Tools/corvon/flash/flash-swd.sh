#!/usr/bin/env bash
#
# CORVON G1 / G2 bare-board SWD flash, with an end-to-end readback check.
#
# Verified on hardware 2026-08-19: DW608 (G1), ST-Link/V2-1 firmware V2J42M27,
# OpenOCD 0.12.0, flying-lead probes on the "G V C D" pads.
#
# Two things here look wrong and are not:
#
# 1. It uses the stock hla driver rather than our own corvon-f405-swd.cfg. A
#    virgin part has blank flash, so the core double-faults on the first fetch
#    and sits in lockup. The dapdirect_swd path treats the resulting poll error
#    as fatal and aborts with "init mode failed (unable to connect)"; hla
#    tolerates it and attaches. The first flash of a bare board can only go
#    this way. corvon-f405-swd.cfg stays the BOR fixture target, where the part
#    already runs firmware.
#
# 2. It connects at 480 kHz. 1800 kHz does not survive the initial handshake on
#    flying leads. Costs nothing: OpenOCD raises the clock to 4 MHz by itself
#    once attached, which is where the flash time actually goes.
#
# Powering - only one source at a time:
#   - 5V on the 4P CAN connector: the whole board, GNSS included. Required for
#     any test that needs a fix, and for the BOR step.
#   - 3V3 on the SWD pad "V": MCU only (~40-50mA). The GNSS sits on 3V3GPS off
#     U7, fed from VCC, so it stays dark - `ver` works, `test` and `mag` do not.
# Never both: the probe supply and the board's own LDO output fight each other.

set -euo pipefail

usage() {
    cat >&2 <<EOF
usage: $(basename "$0") <g1|g2> [--probe-only] [--no-bor] [--dir <dir>]
                       [--speed <kHz>] [--silicon st|geehy|gd]

  --probe-only   read silicon identity, option bytes and flash state, write nothing
  --no-bor       skip the BOR step (engineering only - a shipped board needs it)
  --dir          where the .hex / .apj live (default: this script's directory)
  --speed        SWD connect speed in kHz (default: 480)
  --silicon      st | geehy | gd, for the BOR step (default: geehy)
EOF
    exit 2
}

[ $# -ge 1 ] || usage
BOARD=$(echo "$1" | tr '[:upper:]' '[:lower:]'); shift
case "$BOARD" in
    g1) BOARD_UC=G1; EXPECT_ID=1963 ;;
    g2) BOARD_UC=G2; EXPECT_ID=1964 ;;
    *)  usage ;;
esac

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPEED=480
PROBE_ONLY=0
DO_BOR=1
SILICON=geehy
while [ $# -gt 0 ]; do
    case "$1" in
        --probe-only) PROBE_ONLY=1; shift ;;
        --no-bor)     DO_BOR=0; shift ;;
        --dir)     DIR="$2"; shift 2 ;;
        --speed)   SPEED="$2"; shift 2 ;;
        --silicon) SILICON="$2"; shift 2 ;;
        *) usage ;;
    esac
done

# the BOR configs live next to this script in the firmware tree, and are
# copied alongside it into a release bundle
BOR_DIR="$DIR"
[ -f "$BOR_DIR/bor_common.cfg" ] || BOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../bor" && pwd)"
if [ "$DO_BOR" = 1 ] && [ ! -f "$BOR_DIR/bor_common.cfg" ]; then
    echo "BOR configs not found; pass --no-bor to skip, or put them next to this script" >&2
    exit 1
fi

HEX=$(ls "$DIR"/CORVON-${BOARD_UC}_AP_Periph_with_bl_*.hex 2>/dev/null | head -1 || true)
APJ=$(ls "$DIR"/CORVON-${BOARD_UC}_AP_Periph_[0-9a-f]*.apj  2>/dev/null | head -1 || true)
# --probe-only touches neither artifact, and the whole point of it is to read a
# board on a machine that may not have the release bundle at all.
if [ "$PROBE_ONLY" = 0 ] && { [ -z "$HEX" ] || [ -z "$APJ" ]; }; then
    echo "no CORVON-${BOARD_UC} .hex / .apj found in $DIR" >&2; exit 1
fi

OOCD=(openocd -f interface/stlink.cfg
      -c "transport select hla_swd" -c "adapter speed ${SPEED}"
      -f target/stm32f4x.cfg -c init -c halt)

if [ -n "$HEX" ]; then LABEL=$(basename "$HEX"); else LABEL="probe only, no artifact"; fi
echo "=== ${BOARD_UC}: ${LABEL} ==="
echo "    expecting board_id ${EXPECT_ID}"

# ---- read-only state, before touching anything -----------------------------
"${OOCD[@]}" \
  -c "echo {--- DBGMCU_IDCODE ---}"  -c "mdw 0xE0042000 1" \
  -c "echo {--- FLASH_SIZE KB ---}"  -c "mdh 0x1FFF7A22 1" \
  -c "echo {--- UID 96-bit ---}"     -c "mdw 0x1FFF7A10 3" \
  -c "echo {--- FLASH_OPTCR ---}"    -c "mdw 0x40023C14 2" \
  -c "echo {--- flash head ---}"     -c "mdw 0x08000000 4" \
  -c shutdown

if [ "$PROBE_ONLY" = 1 ]; then echo "probe only, nothing written"; exit 0; fi

# ---- flash -----------------------------------------------------------------
"${OOCD[@]}" -c "program \"$HEX\" verify" -c "reset run" -c shutdown

# ---- read the application region back and compare with the .apj ------------
# "Verified OK" above is OpenOCD checking its own write. This is the
# independent check: what the silicon holds, against the artifact we ship.
APP_BASE=0x08010000
SIZE=$(python3 -c "import json;print(json.load(open('$APJ'))['image_size'])")
# a bare -t prefix is BSD-only; GNU mktemp wants X's in the template, and it
# would fail here *after* the board has already been programmed, silently
# skipping the read-back comparison below
RB=$(mktemp "${TMPDIR:-/tmp}/corvon_rb.XXXXXX")
trap 'rm -f "$RB"' EXIT

"${OOCD[@]}" -c "dump_image \"$RB\" ${APP_BASE} ${SIZE}" -c shutdown

python3 - "$APJ" "$RB" "$EXPECT_ID" <<'PY'
import sys, json, base64, zlib, hashlib, re
apj  = json.load(open(sys.argv[1]))
img  = zlib.decompress(base64.b64decode(apj['image']))
chip = open(sys.argv[2], 'rb').read()
want = int(sys.argv[3])
ok   = True

print("\n=== readback vs artifact ===")
print("  apj  : board_id=%s git_identity=%s image_size=%s md5=%s"
      % (apj.get('board_id'), apj.get('git_identity'), apj.get('image_size'),
         hashlib.md5(img).hexdigest()))
print("  chip : %d bytes md5=%s" % (len(chip), hashlib.md5(chip).hexdigest()))

if apj.get('board_id') != want:
    print("  FAIL board_id %s != %d" % (apj.get('board_id'), want)); ok = False
if img != chip:
    d = [i for i in range(min(len(img), len(chip))) if img[i] != chip[i]]
    print("  FAIL byte compare: %d differ, first at 0x%X" % (len(d), d[0] if d else -1))
    ok = False
else:
    print("  PASS byte-for-byte identical")

# strings taken out of the silicon, not typed by hand
m = re.search(rb'org\.ardupilot\.[A-Za-z0-9\-]+', chip)
print("  node name in silicon: %s" % (m.group().decode() if m else "NOT FOUND"))
if not m: ok = False

sys.exit(0 if ok else 1)
PY

if [ "$DO_BOR" = 1 ]; then
    # The probe is already on the pads and the part is already running our
    # firmware, so this costs a couple of seconds and saves a separate station.
    # Writing option bytes is the one step with no BOR protection of its own:
    # do not cut power or lift the probe while it runs. A QUARANTINE_HOLD_POWER
    # means stop and keep the board powered - never retry automatically.
    echo
    echo "=== BOR level 3 (silicon: $SILICON) ==="
    BOROOCD=(openocd -f interface/stlink-dap.cfg -f "$BOR_DIR/corvon-f405-swd.cfg"
             -c "set SILICON $SILICON" -f "$BOR_DIR/bor_common.cfg")
    "${BOROOCD[@]}" -f "$BOR_DIR/bor_program.cfg"
    "${BOROOCD[@]}" -f "$BOR_DIR/bor_check.cfg"
fi

echo
echo "=== ${BOARD_UC} flashed and verified ==="
if [ "$DO_BOR" = 1 ]; then
    echo "BOR level 3 written and checked against flash storage."
    echo "bor_check reads the stored option bytes rather than the loaded copy, so"
    echo "it needs no power cycle - see Tools/corvon/bor/README.md for the one"
    echo "calibration that still owes a measured power cycle."
else
    echo "BOR SKIPPED - this board is not shippable until it has been set."
fi
echo "next: power the board from the 4P connector, wait 8s, then console at"
echo "57600 8N1: ver / test / mag"
