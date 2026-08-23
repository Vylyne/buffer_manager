#!/bin/bash
# buffer_manager uninstall.sh — reverses install.sh's symlinks.
#
# Does not touch the Makefile line (harmless if buffer.c is gone - the
# Klipper build just won't reference a file that no longer exists) or your
# printer.cfg, since install.sh never wrote either of those for you either.
set -euo pipefail

KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"
KLIPPER_EXTRAS="$KLIPPER_DIR/klippy/extras"
KLIPPER_SRC="$KLIPPER_DIR/src"

unlink_if_symlink() {
    local path="$1"
    if [ -L "$path" ]; then
        echo "Removing $path"
        rm "$path"
    elif [ -e "$path" ]; then
        echo "SKIPPING $path - exists but is not a symlink (not touching it)" >&2
    fi
}

unlink_if_symlink "$KLIPPER_EXTRAS/lll_buffer.py"
unlink_if_symlink "$KLIPPER_EXTRAS/buffer_manager.py"
unlink_if_symlink "$KLIPPER_SRC/buffer.c"

echo
echo "Done. The Makefile line for buffer.c was left in place - remove it by"
echo "hand from $KLIPPER_SRC/Makefile if you want a completely clean tree."
