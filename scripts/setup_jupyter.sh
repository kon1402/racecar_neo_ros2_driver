#!/bin/bash
# JupyterLab — install + workspace setup for racecar student notebooks.
# Idempotent: re-runs skip work already done.
set -e

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"
JUPYTER_WS="$USER_HOME/jupyter_ws"

# PEP 668 → per-user install. jupyterlab pulls jupyter_server, ipykernel,
# tornado, et al; ~100 MB total on disk.
if ! command -v "$USER_HOME/.local/bin/jupyter" >/dev/null 2>&1; then
    pip3 install --user --break-system-packages jupyterlab
fi

# Notebook root. Empty unless we ship example notebooks later.
if [ ! -d "$JUPYTER_WS" ]; then
    mkdir -p "$JUPYTER_WS"
    cat > "$JUPYTER_WS/README.md" <<'EOF'
# RACECAR Neo Jupyter Workspace

JupyterLab serves this directory at http://<robot>:8888 when
racecar-jupyter.service is running.

Start a notebook and `import rclpy` — the systemd unit pre-sets
PYTHONPATH/AMENT_PREFIX_PATH/LD_LIBRARY_PATH so ROS2 messages and the
racecar driver are importable.
EOF
    echo "Created $JUPYTER_WS"
fi
