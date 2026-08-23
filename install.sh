#!/bin/bash
# buffer_manager install.sh
#
# Symlinks the host-side Python modules and the MCU source file into a
# Klipper checkout, and reminds the Makefile of buffer.c. Run this on the
# Klipper host (e.g. the Raspberry Pi), not on the machine you edit on.
#
# THIS SCRIPT NEVER FLASHES ANYTHING. Building and flashing the firmware are
# separate, deliberate, manual steps you run yourself - see README.md
# "Install & update" and CLAUDE.md. A flash-everything script is a hazard the
# moment more than one board on the bus shares this MCU; that risk is not
# worth automating away.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"
KLIPPER_EXTRAS="$KLIPPER_DIR/klippy/extras"
KLIPPER_SRC="$KLIPPER_DIR/src"
MAKEFILE="$KLIPPER_SRC/Makefile"
MAKEFILE_LINE='src-$(CONFIG_MACH_STM32F072) += buffer.c'

if [ ! -d "$KLIPPER_EXTRAS" ] || [ ! -d "$KLIPPER_SRC" ]; then
    echo "ERROR: $KLIPPER_DIR does not look like a Klipper checkout" >&2
    echo "       (expected klippy/extras/ and src/ underneath it)." >&2
    echo "       Set KLIPPER_DIR=/path/to/klipper if it lives elsewhere." >&2
    exit 1
fi

link() {
    local src="$1" dst="$2"
    if [ ! -f "$src" ]; then
        echo "ERROR: source file not found: $src" >&2
        exit 1
    fi
    echo "Linking $(basename "$src") -> $dst"
    ln -sf "$src" "$dst"
}

link "$SCRIPT_DIR/klippy/extras/lll_buffer.py"    "$KLIPPER_EXTRAS/lll_buffer.py"
link "$SCRIPT_DIR/klippy/extras/buffer_manager.py" "$KLIPPER_EXTRAS/buffer_manager.py"
link "$SCRIPT_DIR/src/buffer.c"                    "$KLIPPER_SRC/buffer.c"

if [ -f "$MAKEFILE" ] && grep -qF "$MAKEFILE_LINE" "$MAKEFILE"; then
    echo "Makefile already lists buffer.c - nothing to add."
elif [ -f "$MAKEFILE" ]; then
    echo "Adding buffer.c to $MAKEFILE"
    printf '\n%s\n' "$MAKEFILE_LINE" >> "$MAKEFILE"
    echo "NOTE: 'git pull' on the Klipper tree reverts this line - rerun this"
    echo "      script (or re-add it by hand) after updating Klipper."
else
    echo "WARNING: $MAKEFILE not found - add this line to it by hand:" >&2
    echo "  $MAKEFILE_LINE" >&2
fi

cat <<'EOF'

Done linking. Two things this script deliberately did NOT do:

  1. Build or flash anything. See README.md "Install & update" and
     CLAUDE.md for the build steps (they run on this host) and the
     Katapult flashing procedure (manual, one board at a time, on purpose).

  2. Touch printer.cfg. Copy the sections you need from
     config/printer.cfg.example, config/lll_macros.cfg.example and
     config/lcd_menu_lll.cfg.example into your own config and edit the
     pin names, MCU section name, and sensor names for your machine before
     including them. These are templates, not something safe to symlink
     blindly across machines.

After building and flashing, restart Klipper with FIRMWARE_RESTART.
EOF
