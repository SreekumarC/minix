#!/usr/bin/env bash
# Sets up the Jetson side: venv, dependencies, systemd service.
# Run from the repo root: ./scripts/install.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JETSON_DIR="$REPO_DIR/jetson"

echo "==> Creating venv"
python3 -m venv "$JETSON_DIR/venv"
source "$JETSON_DIR/venv/bin/activate"

echo "==> Installing dependencies"
pip install --upgrade pip
pip install -r "$JETSON_DIR/requirements.txt"

echo "==> Installing systemd service"
sudo cp "$JETSON_DIR/minix.service" /etc/systemd/system/minix.service
sudo systemctl daemon-reload
sudo systemctl enable minix.service

echo "==> Done."
echo "Edit /etc/systemd/system/minix.service if User/WorkingDirectory don't match this machine,"
echo "then: sudo systemctl start minix   (or reboot)"
echo "Logs: journalctl -u minix -f"
