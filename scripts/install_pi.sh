#!/usr/bin/env bash
set -euo pipefail

python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[hardware]"

echo "Install Picamera2 with: sudo apt install -y python3-picamera2"
echo "Add your user to dialout with: sudo usermod -aG dialout \$USER"
echo "Then reboot before using /dev/ttyUSB0."
