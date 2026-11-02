#!/usr/bin/env bash
# Build an isolated, native-only development dependency and its robot consumer.
set -euo pipefail
robot_root="$(cd "$(dirname "$0")/.." && pwd)"
base_root="${MECO_FAST_BASE_ROOT:-$(cd "$robot_root/../mecobase-fast-control" && pwd)}"
sim_root="${MECO_FAST_SIM_ROOT:-$(cd "$robot_root/../mecosim-fast-control" && pwd)}"
export JAVA_HOME="${JAVA_HOME:-/home/brian/wpilib/2026/jdk}"
cd "$base_root"
timeout 180 ./gradlew frcBaseLinuxx86-64ReleaseStaticLibrary --max-workers=2 --console=plain
cd "$robot_root"
python3 scripts/package_fast_base.py "$base_root" build/fast-base-maven
cd "$sim_root/adapters/wpilib-2026"
timeout 180 ./gradlew mecoSimAdapterReleaseStaticLibrary -PmecoBaseRoot="$base_root" --max-workers=2 --console=plain
cd "$robot_root"
python3 scripts/package_fast_sim.py "$sim_root" build/fast-base-maven
base_version="$(python3 -c 'import json; print(json.load(open("build/fast-base-maven/provenance.json"))["version"])')"
sim_version="$(python3 -c 'import json; print(json.load(open("build/fast-base-maven/simulation-provenance.json"))["version"])')"
timeout 180 ./gradlew installFrcUserProgramLinuxx86-64ReleaseExecutable \
  installFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe \
  -PmecoBaseMaven="$robot_root/build/fast-base-maven" \
  -PmecoBaseVersion="$base_version" -PfastControlApi=true \
  -PmecoSimulationMaven="$robot_root/build/fast-base-maven" -PmecoSimulationVersion="$sim_version" \
  --max-workers=2 --console=plain
