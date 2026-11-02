"""Package one native release development artifact; never alter public releases."""
import argparse
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source", type=Path)
parser.add_argument("repository", type=Path)
args = parser.parse_args()
source, repository = args.source.resolve(), args.repository.resolve()
revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip()
diff = subprocess.check_output(["git", "diff", "HEAD"], cwd=source, text=True)
version = "2026.2.12-dev.fast-control." + hashlib.sha256((revision + diff).encode()).hexdigest()[:12]
artifact = "meco-base-cpp"
target = repository / "com/mecorobotics/frc" / artifact / version
target.mkdir(parents=True, exist_ok=True)
library = source / "build/libs/frcBase/static/linuxx86-64/release/libfrcBase.a"
if not library.is_file():
    raise SystemExit(f"Build the native release static library first: {library}")
prefix = target / f"{artifact}-{version}"
with zipfile.ZipFile(str(prefix) + "-headers.zip", "w", zipfile.ZIP_DEFLATED) as out:
    for path in sorted((source / "src/main/include/base").rglob("*")):
        if path.is_file():
            out.write(path, path.relative_to(source / "src/main/include"))
with zipfile.ZipFile(str(prefix) + "-linuxx86-64static.zip", "w", zipfile.ZIP_DEFLATED) as out:
    out.write(library, "linux/x86-64/static/libmecoBase.a")
    out.write(source / "LICENSE.md", "LICENSE.md")
Path(str(prefix) + ".pom").write_text(
    '<project xmlns="http://maven.apache.org/POM/4.0.0"><modelVersion>4.0.0</modelVersion>'
    f'<groupId>com.mecorobotics.frc</groupId><artifactId>{artifact}</artifactId>'
    f'<version>{version}</version><packaging>pom</packaging></project>\n'
)
revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip()
diff = subprocess.check_output(["git", "diff", "HEAD"], cwd=source, text=True)
(repository / "provenance.json").write_text(json.dumps({
    "version": version, "source": str(source), "revision": revision,
    "scope": "native Linux release only; not a published or hardware-qualified package",
    "dirty": bool(diff),
}, indent=2) + "\n")
(repository / "source.patch").write_text(diff)
print(repository)
