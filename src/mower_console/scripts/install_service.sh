#!/usr/bin/env bash
set -eo pipefail
test "$(uname -m)" = aarch64
source /opt/ros/humble/setup.bash
source /home/cat/MobileEss_ws/install/setup.bash
console_share=$(ros2 pkg prefix --share mower_console)
test -s "$console_share/web/dist/index.html"
test -s "$console_share/config/lattice_r4_10cm.json"
sudo install -m 0644 "$console_share/systemd/mower-console.service" /etc/systemd/system/mower-console.service
sudo systemctl daemon-reload
sudo systemctl enable --now mower-console.service
