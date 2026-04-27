#!/bin/bash
# install_controller_rules.sh
# Installs a udev rules file for a gamepad/controller so it is accessible
# without root and joy_node can read it.
#
# Usage (run once on the ROBOT's Linux machine):
#   chmod +x install_controller_rules.sh
#   ./install_controller_rules.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_DIR="$SCRIPT_DIR/controller_rules"

echo "============================================"
echo "  Controller udev rules — installer"
echo "============================================"
echo ""

# ── 1. Discover available .rules files ───────────────────────────────────────
if [ ! -d "$RULES_DIR" ]; then
    echo "✗  Rules directory not found: $RULES_DIR"
    exit 1
fi

mapfile -t RULES_FILES < <(find "$RULES_DIR" -maxdepth 1 -name "*.rules" | sort)

if [ ${#RULES_FILES[@]} -eq 0 ]; then
    echo "✗  No .rules files found in $RULES_DIR"
    exit 1
fi

# ── 2. Ask the user which file to install ────────────────────────────────────
echo "Available rules files:"
echo ""
for i in "${!RULES_FILES[@]}"; do
    echo "  [$((i+1))]  $(basename "${RULES_FILES[$i]}")"
done
echo ""

while true; do
    read -rp "Select a file to install [1-${#RULES_FILES[@]}]: " CHOICE
    if [[ "$CHOICE" =~ ^[0-9]+$ ]] && [ "$CHOICE" -ge 1 ] && [ "$CHOICE" -le "${#RULES_FILES[@]}" ]; then
        break
    fi
    echo "  Invalid choice. Please enter a number between 1 and ${#RULES_FILES[@]}."
done

RULES_SRC="${RULES_FILES[$((CHOICE-1))]}"
RULES_FILENAME="$(basename "$RULES_SRC")"
RULES_DEST="/etc/udev/rules.d/$RULES_FILENAME"

echo ""
echo "Source : $RULES_SRC"
echo "Target : $RULES_DEST"
echo ""

# ── 3. Install rules file ─────────────────────────────────────────────────────
echo "→ Copying rules file (requires sudo)..."
sudo cp "$RULES_SRC" "$RULES_DEST"
sudo chmod 644 "$RULES_DEST"
echo "✓  Rules file installed."
echo ""

# ── 4. Reload udev ────────────────────────────────────────────────────────────
echo "→ Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✓  udev rules reloaded."
echo ""

# ── 5. Ensure user is in the required groups ──────────────────────────────────
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

# ── 6. Print final instructions ───────────────────────────────────────────────
if [ ${#ADDED_GROUPS[@]} -gt 0 ]; then
    echo "⚠  Group membership changed: ${ADDED_GROUPS[*]}"
    echo "   You must LOG OUT and back in (or run 'newgrp plugdev') for the"
    echo "   group changes to take effect before plugging in the controller."
    echo ""
fi

echo "============================================"
echo "  Done. Installed: $RULES_FILENAME"
echo "  Reconnect your controller and verify with:"
echo "    ls -l /dev/input/js*"
echo "============================================"
