#!/bin/bash
set -e

# ==========================================
# Configuration
# ==========================================
# Set this to the original entrypoint of your base image (e.g., "/ros_entrypoint.sh").
# Leave it empty ("") if the base image has no specific entrypoint.
ORIGINAL_ENTRYPOINT="/ros_entrypoint.sh"

# ==========================================
# X11 / Wayland Authorization Setup
# Assumption: Host's XAUTHORITY file is located within XDG_RUNTIME_DIR.
# ==========================================
# Dynamically find the auth file (.mutter-Xwaylandauth.* or *Xauthority*)
AUTH_FILE=$(find /tmp/runtime_dir -maxdepth 2 -type f \( -name ".mutter-Xwaylandauth.*" -o -name "*Xauthority*" \) 2>/dev/null | head -n 1)

if [ -n "$AUTH_FILE" ]; then
    export XAUTHORITY="$AUTH_FILE"
else
    echo "Warning: No Xauthority or Mutter Wayland auth file found in /tmp/runtime_dir."
fi

# ==========================================
# Entrypoint Chaining
# ==========================================
if [ -n "$ORIGINAL_ENTRYPOINT" ] && [ -f "$ORIGINAL_ENTRYPOINT" ]; then
    exec "$ORIGINAL_ENTRYPOINT" "$@"
else
    exec "$@"
fi
