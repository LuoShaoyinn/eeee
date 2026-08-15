#!/bin/sh
set -eu

TARGET_DIR="$1"
if [ -L "$TARGET_DIR/etc/dropbear" ]; then
    rm -f "$TARGET_DIR/etc/dropbear"
fi
mkdir -p "$TARGET_DIR/etc/wpa_supplicant" "$TARGET_DIR/etc/dropbear"
chmod 700 "$TARGET_DIR/etc/dropbear"
rm -f "$TARGET_DIR/etc/dropbear/dropbear_*"

# Keep SSH root-only. Dropbear generates host keys on first boot.
cat > "$TARGET_DIR/etc/dropbear/dropbear.conf" <<'EOF'
NO_START=0
DROPBEAR_PORT=22
DROPBEAR_EXTRA_ARGS=""
EOF

# This is intentionally a template; credentials are injected before flashing.
if [ ! -f "$TARGET_DIR/etc/wpa_supplicant/wpa_supplicant.conf" ]; then
    cat > "$TARGET_DIR/etc/wpa_supplicant/wpa_supplicant.conf" <<'EOF'
ctrl_interface=/run/wpa_supplicant
update_config=0
country=SG
EOF
fi
