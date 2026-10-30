"""Point repository-file links at GitHub when rendering the documentation site."""

from pathlib import Path
import re
import os
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
REPO_URL = "https://github.com/MECO-Robotics/2026-Rebuilt-cpp"
LINK = re.compile(r"(?P<start>!?(?:\[[^\]]*\])\()(?P<target>[^\s)]+)(?P<end>\))")


def on_page_markdown(markdown, page, **kwargs):
    source = DOCS / page.file.src_uri

    def rewrite(match):
        target = match.group("target")
        url = urlsplit(target)
        if url.scheme or url.netloc or not url.path or url.path.startswith("/"):
            return match.group(0)
        resolved = (source.parent / url.path).resolve()
        if resolved.is_relative_to(DOCS):
            if url.path.startswith("../"):
                destination = Path(os.path.relpath(resolved, source.parent)).as_posix()
                if url.fragment:
                    destination += f"#{url.fragment}"
                return match.group("start") + destination + match.group("end")
            return match.group(0)
        simulation_repo = ROOT.parent / "MecoSimulation-cpp"
        if resolved.is_relative_to(simulation_repo):
            relative = resolved.relative_to(simulation_repo).as_posix()
            return match.group("start") + f"https://github.com/MECO-Robotics/MecoSimulation-cpp/blob/wpilib-2026/{relative}" + match.group("end")
        if not resolved.is_relative_to(ROOT):
            return match.group(0)
        relative = resolved.relative_to(ROOT).as_posix()
        kind = "tree" if resolved.is_dir() else "blob"
        destination = f"{REPO_URL}/{kind}/main/{relative}"
        if url.query:
            destination += f"?{url.query}"
        if url.fragment:
            destination += f"#{url.fragment}"
        return match.group("start") + destination + match.group("end")

    return LINK.sub(rewrite, markdown)
