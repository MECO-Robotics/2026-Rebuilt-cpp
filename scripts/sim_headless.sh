#!/usr/bin/env bash
# Launch only the desktop simulator, with local HAL websocket control for sim_smoke.py.
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
wpilib_root="${WPILIB_ROOT:-$HOME/wpilib/2026}"
ws_zip="$wpilib_root/maven/edu/wpi/first/halsim/halsim_ws_server/2026.2.1/halsim_ws_server-2026.2.1-linuxx86-64.zip"
sim_binary=build/exe/frcUserProgram/linuxx86-64/release/frcUserProgram
sim_libs=build/install/frcUserProgramTest/linuxx86-64/release/lib
if [[ ! -f "$ws_zip" ]]; then
  echo "Install WPILib 2026.2.1 or set WPILIB_ROOT; missing $ws_zip" >&2
  exit 1
fi
if [[ ! -x "$sim_binary" || ! -d "$sim_libs" ]]; then
  echo 'Run ./gradlew build before starting the headless simulator.' >&2
  exit 1
fi
sim_extension_dir="$(mktemp -d "${TMPDIR:-/tmp}/base-bot-halsim.XXXXXX")"
trap 'rm -rf -- "$sim_extension_dir"' EXIT
unzip -q "$ws_zip" -d "$sim_extension_dir"
# Snapshot libraries so a later build cannot truncate shared objects in the running process.
mkdir "$sim_extension_dir/lib"
cp -a --reflink=auto -- "$sim_libs"/*.so* "$sim_extension_dir/lib/"
cp --reflink=auto -- "$sim_binary" "$sim_extension_dir/frcUserProgram"
export LD_LIBRARY_PATH="$sim_extension_dir/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export HALSIM_EXTENSIONS="$sim_extension_dir/linux/x86-64/shared/libhalsim_ws_server.so"
if [[ "${FRC_SIM_GUI:-0}" == 1 ]]; then
  # SimGUI polls physical joysticks on its render thread. Mesa vsync can
  # throttle covered Xwayland windows to 1 Hz, starving input updates.
  export vblank_mode="${vblank_mode:-0}"
  gui_zip="$wpilib_root/maven/edu/wpi/first/halsim/halsim_gui/2026.2.1/halsim_gui-2026.2.1-linuxx86-64.zip"
  unzip -q "$gui_zip" -d "$sim_extension_dir/gui"
  export HALSIM_EXTENSIONS="$HALSIM_EXTENSIONS:$sim_extension_dir/gui/linux/x86-64/shared/libhalsim_gui.so"
fi
"$sim_extension_dir/frcUserProgram"
