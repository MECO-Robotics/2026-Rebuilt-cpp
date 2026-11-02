"""Rebuild-dependent Simulation adapter package, retaining unmodified published Jolt objects."""
import argparse
import hashlib
import json
import shutil
import subprocess
import zipfile
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("simulation", type=Path)
p.add_argument("repository", type=Path)
a = p.parse_args()
root, repo = a.simulation.resolve(), a.repository.resolve()
base = json.loads((repo / "provenance.json").read_text())
adapter = root / "adapters/wpilib-2026/build/libs/mecoSimAdapter/static/release/libmecoSimAdapter.a"
cache = Path.home() / ".gradle/caches/modules-2/files-2.1/com.mecorobotics.sim/meco-simulation-wpilib-cpp/2026.0.16"
binary = next(cache.glob("*/meco-simulation-wpilib-cpp-2026.0.16-linuxx86-64static.zip"))
headers = next(cache.glob("*/meco-simulation-wpilib-cpp-2026.0.16-headers.zip"))
version = "2026.0.17-dev.fast-control." + hashlib.sha256(
    (base["version"] + hashlib.sha256(adapter.read_bytes()).hexdigest()).encode()
).hexdigest()[:12]
stage = repo / "simulation-stage"
stage.mkdir(parents=True, exist_ok=True)
core = stage / "published-core.a"
with zipfile.ZipFile(binary) as z:
    core.write_bytes(z.read("linux/x86-64/static/libmecoSimAdapter.a"))
members = subprocess.check_output(["ar", "t", str(adapter)], text=True).splitlines()
old_members = subprocess.check_output(["ar", "t", str(core)], text=True).splitlines()
if set(members) != {"PracticeMatchController.o", "DriveSimulationAdapter.o", "DriveHardwareSimulation.o"}:
    raise SystemExit("Unexpected adapter object list; inspect compatibility before packaging")
if not all(old_members.count(member) == 1 for member in members):
    raise SystemExit("Published archive does not contain exactly one of each adapter object")
subprocess.run(["ar", "d", str(core), *members], check=True)
combined = stage / "libmecoSimAdapter.a"
subprocess.run(["ar", "-M"], input=f"create {combined}\naddlib {core}\naddlib {adapter}\nsave\nend\n", text=True, check=True)
target = repo / "com/mecorobotics/sim/meco-simulation-wpilib-cpp" / version
target.mkdir(parents=True, exist_ok=True)
prefix = target / f"meco-simulation-wpilib-cpp-{version}"
with zipfile.ZipFile(str(prefix) + "-linuxx86-64static.zip", "w", zipfile.ZIP_DEFLATED) as z:
    z.write(combined, "linux/x86-64/static/libmecoSimAdapter.a")
    z.write(root / "LICENSE.md", "LICENSE.md")
    z.write(root / "Jolt-LICENSE", "Jolt-LICENSE")
shutil.copyfile(headers, str(prefix) + "-headers.zip")
Path(str(prefix) + ".pom").write_text(
    '<project xmlns="http://maven.apache.org/POM/4.0.0"><modelVersion>4.0.0</modelVersion>'
    '<groupId>com.mecorobotics.sim</groupId><artifactId>meco-simulation-wpilib-cpp</artifactId>'
    f'<version>{version}</version><packaging>pom</packaging></project>\n'
)
(repo / "simulation-provenance.json").write_text(json.dumps({
    "version": version, "baseVersion": base["version"], "source": str(root),
    "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
    "retainedCoreArchiveSha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
    "adapterSha256": hashlib.sha256(adapter.read_bytes()).hexdigest(),
    "scope": "Native release development adapter rebuilt against the development Base headers; unchanged published Jolt core",
}, indent=2) + "\n")
print(version)
