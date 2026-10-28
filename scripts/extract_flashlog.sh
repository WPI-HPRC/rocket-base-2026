#!/usr/bin/env bash
# Pulls the on-chip flash log off a board in DFU mode, decodes every session,
# then clears the log on the board so the space is free for the next flight.
#
# Put the board in DFU mode first (hold BOOT while plugging in USB) so the
# flight firmware doesn't run and start a new session.
#
# Usage: scripts/extract_flashlog.sh [--keep] [out_dir]
#   --keep   don't clear the log on the board
#   out_dir  defaults to flashlog_<date>_<time>
# Result in out_dir:
#   flashlog.bin   raw copy of all of flash bank 2 (table + data)
#   table.txt      every allocation table record and sector owner
#   sessionN/      flightData.fb.bin, debugLog.txt, decoded/*.csv
#
# Clearing: the DFU bootloader can only erase sectors it writes to, and
# writing into a data sector would leave words the firmware can't program
# again. So only the table sector is rewritten, with a fresh table that keeps
# the session numbering. On its next boot the firmware sees sectors 1-7 as
# free but not blank and erases them in the background (~1-2 s each).

set -euo pipefail

KEEP=0
if [ "${1:-}" = "--keep" ]; then
  KEEP=1
  shift
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-flashlog_$(date +%Y%m%d_%H%M%S)}"
BANK2=0x08100000

if command -v dfu-util >/dev/null; then
  DFU=dfu-util
elif [ -x "$HOME/.platformio/packages/tool-dfuutil/bin/dfu-util" ]; then
  DFU="$HOME/.platformio/packages/tool-dfuutil/bin/dfu-util"
else
  echo "dfu-util not found (install it, or build once with PlatformIO)" >&2
  exit 1
fi

# STM32 system bootloader
if ! "$DFU" -l 2>/dev/null | grep "0483:df11" >/dev/null; then
  echo "No board in DFU mode found. Hold BOOT while plugging in USB and try again." >&2
  exit 1
fi

mkdir -p "$OUT"
echo "Reading flash bank 2..."
"$DFU" -d 0483:df11 -a 0 -s "$BANK2:1048576" -U "$OUT/flashlog.bin" >/dev/null
# Read twice so a bad read is caught before anything is erased
"$DFU" -d 0483:df11 -a 0 -s "$BANK2:1048576" -U "$OUT/.verify.bin" >/dev/null
if ! cmp -s "$OUT/flashlog.bin" "$OUT/.verify.bin"; then
  echo "The two reads of the flash differ, not trusting this dump. Nothing was erased." >&2
  exit 1
fi
rm "$OUT/.verify.bin"

python3 -I "$ROOT/scripts/flashlog_extract.py" "$OUT/flashlog.bin" "$OUT"

echo "Building decoder..."
(cd "$ROOT" && pio run -s -e flatbuffer_decoder)
DECODER="$ROOT/.pio/build/flatbuffer_decoder/program"

for session in "$OUT"/session*/; do
  session="${session%/}"
  [ -s "$session/flightData.fb.bin" ] || continue
  # The decoder writes into ./decoded
  (cd "$session" && "$DECODER" flightData.fb.bin >/dev/null)
  echo "Decoded $session"
  wc -l "$session"/decoded/*.csv | sed 's/^/  /'
done

if [ "$KEEP" = 1 ]; then
  echo "--keep given, the log on the board was left as is."
  exit 0
fi

echo "Clearing the log on the board..."
"$DFU" -d 0483:df11 -a 0 -s "$BANK2" -D "$OUT/reset_table.bin" >/dev/null 2>&1
"$DFU" -d 0483:df11 -a 0 -s "$BANK2:64" -U "$OUT/.check.bin" >/dev/null
# Expect the fresh table record followed by a blank word
if cmp -s "$OUT/.check.bin" <(cat "$OUT/reset_table.bin"; head -c 32 /dev/zero | LC_ALL=C tr '\0' '\377'); then
  rm "$OUT/.check.bin" "$OUT/reset_table.bin"
  echo "Log cleared. The firmware erases the old data sectors on its next boot."
else
  echo "Clearing failed, the table on the board doesn't match. Data is saved in $OUT." >&2
  exit 1
fi
