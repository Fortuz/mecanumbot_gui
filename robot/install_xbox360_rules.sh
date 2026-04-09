#!/bin/bash
# install_xbox360_rules.sh
# Installs the Xbox 360 wireless adapter udev rules so the controller
# is accessible without root and joy_node / xpad can read it.
#
# Usage (run once on the ROBOT's Linux machine):
#   chmod +x install_xbox360_rules.sh
#   ./install_xbox360_rules.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="$SCRIPT_DIR/controller_rules/99-xbox360-wireless.rules"
RULES_DEST="/etc/udev/rules.d/99-xbox360-wireless.rules"

echo "============================================"
echo "  Xbox 360 Wireless Adapter — udev install"
echo "============================================"
echo ""

# ── 1. Verify source file ─────────────────────────────────────────────────────
if [ ! -f "$RULES_SRC" ]; then
    echo "✗  Rules file not found: $RULES_SRC"
    echo "   Make sure you are running this script from inside the project directory."
    exit 1
fi

echo "Source : $RULES_SRC"
echo "Target : $RULES_DEST"
echo ""

# ── 2. Install rules file ─────────────────────────────────────────────────────
echo "→ Copying rules file (requires sudo)..."
sudo cp "$RULES_SRC" "$RULES_DEST"
sudo chmod 644 "$RULES_DEST"
echo "✓  Rules file installed."
echo ""

# ── 3. Reload udev ────────────────────────────────────────────────────────────
echo "→ Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✓  udev rules reloaded."
echo ""

# ── 4. Ensure user is in the required groups ──────────────────────────────────
REQUIRED_GROUPS=(plugdev input)
ADDED_GROUPS=()

for GRP in "${REQUIRED_GROUPS[@]}"; do
    if id -nG "$USER" | grep -qw "$GRP"; then
        echo "✓  User '$USER' is already in group '$GRP'."
    else
        echo "→  Adding '$USER' to group '$GRP'..."
        sudo usermod -aG "$GRP" "$USER"
        ADDED_GROUPS+=("$GRP")
        echo "✓  Added."
    fi
done

echo ""

# ── 5. Print final instructions ───────────────────────────────────────────────
if [ ${#ADDED_GROUPS[@]} -gt 0 ]; then
    echo "⚠  Group membership changed: ${ADDED_GROUPS[*]}"
    echo "   You must LOG OUT and back in (or run 'newgrp plugdev') for the"
    echo "   group changes to take effect before plugging in the adapter."
    echo ""
fi

echo "============================================"
echo "  Done."
echo "  Reconnect the Xbox 360 wireless adapter."
echo "  Then verify with:  ls -l /dev/input/js*"
echo "============================================"
