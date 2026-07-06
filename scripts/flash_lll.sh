#!/bin/bash
# Flash one LLL Plus board over USB via Katapult (no case opening, no DFU).
# Usage: ./flash_lll.sh <chip-serial>   (e.g. 3C003A000957465331323720)
# Assumes: ~/klipper (built: out/klipper.bin), ~/katapult, ~/klippy-env.
# The board must currently run Klipper or Katapult. One board at a time.
SER="$1"
PY="$HOME/klippy-env/bin/python"
FT="$HOME/katapult/scripts/flashtool.py"
BIN="$HOME/klipper/out/klipper.bin"
KDEV="/dev/serial/by-id/usb-Klipper_stm32f072xb_${SER}-if00"

if [ -z "$SER" ]; then echo "usage: $0 <chip-serial>"; exit 1; fi

echo "=== [$SER] request bootloader ==="
$PY $FT -d "$KDEV" -r 2>&1 | tail -3
KAT=""
for i in $(seq 1 15); do
  KAT=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i "katapult" | head -1)
  if [ -n "$KAT" ]; then echo "katapult up (${i}s): $KAT"; break; fi
  sleep 1
done
if [ -z "$KAT" ]; then echo "ERROR: katapult did not enumerate for $SER"; exit 1; fi

echo "=== [$SER] flashing $BIN ==="
$PY $FT -d "/dev/serial/by-id/$KAT" -f "$BIN" 2>&1 | tail -20

echo "=== [$SER] waiting for Klipper to come back ==="
for i in $(seq 1 15); do
  D=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i "Klipper_stm32f072xb_${SER}")
  if [ -n "$D" ]; then echo "Klipper back (${i}s): $D"; break; fi
  sleep 1
done
