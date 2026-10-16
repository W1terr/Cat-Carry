r"""Builds "Cat Carry <version>.zip" for a mod manager: SKSE\Plugins\CatCarry.dll, .pdb and everything in res\core
(CatCarry.ini, translations, the animal files of every supported mod: one for a mod that isn't installed does nothing).
The version is the target's set_version in xmake.lua (a trailing ".0" is left out of the name).

usage: python package.py <build dir> <output dir>"""
import re
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "res" / "core"
BUILD = Path(sys.argv[1])
OUTPUT = Path(sys.argv[2])

text = (ROOT / "xmake.lua").read_text(encoding="utf-8")
version = re.search(r'target\("CatCarry"\)\s*\n\s*set_version\("([^"]+)"\)', text).group(1)
version = re.sub(r"\.0$", "", version)
dll = next(BUILD.rglob("releasedbg/CatCarry.dll"))

files = {
    "SKSE/Plugins/CatCarry.dll": dll,
    "SKSE/Plugins/CatCarry.pdb": dll.with_suffix(".pdb"),
}
files |= {p.relative_to(CORE).as_posix(): p for p in CORE.rglob("*") if p.is_file()}

OUTPUT.mkdir(parents=True, exist_ok=True)
archive = OUTPUT / f"Cat Carry {version}.zip"
with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
    for entry, path in files.items():
        z.write(path, entry)
print(archive)
