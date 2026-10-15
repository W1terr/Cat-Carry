r"""Builds the FOMOD archive "Cat Carry <version>.zip". The version is the target's set_version in xmake.lua (a trailing
".0" is left out of the name).

  00 Core        SKSE\Plugins\CatCarry.dll, .pdb + res\core (CatCarry.ini)
  01 House Cats  res\01 House Cats: the House Cats animal file (always installed, the mod needs House Cats)
  10 Piglet      res\10 Piglet: the Piglet animal file (optional, recommended when piggypet.esp is active)

usage: python package.py <build dir> <output dir>"""
import re
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RES = ROOT / "res"
BUILD = Path(sys.argv[1])
OUTPUT = Path(sys.argv[2])

HOUSE_CATS = "House Cats- Mihail Monsters and Animals"
PIGLET = "Piglet - The courageous piggy companion"

MODULE_CONFIG = f"""<?xml version="1.0" encoding="UTF-8"?>
<config xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://qconsulting.ca/fo3/ModConfig5.0.xsd">
    <moduleName>Cat Carry</moduleName>
    <requiredInstallFiles>
        <folder source="00 Core" destination=""/>
    </requiredInstallFiles>
    <installSteps order="Explicit">
        <installStep name="Animals">
            <optionalFileGroups order="Explicit">
                <group name="Animals you can pick up" type="SelectAny">
                    <plugins order="Explicit">
                        <plugin name="House Cats (required)">
                            <description>The cats of {HOUSE_CATS} (mihailhousecat.esp). Cat Carry needs that mod; this part is always installed.</description>
                            <files>
                                <folder source="01 House Cats" destination="" priority="0"/>
                            </files>
                            <typeDescriptor>
                                <type name="Required"/>
                            </typeDescriptor>
                        </plugin>
                        <plugin name="Piglet">
                            <description>Add-on: pick up Piglet from {PIGLET} (piggypet.esp) too.</description>
                            <files>
                                <folder source="10 Piglet" destination="" priority="0"/>
                            </files>
                            <typeDescriptor>
                                <dependencyType>
                                    <defaultType name="Optional"/>
                                    <patterns>
                                        <pattern>
                                            <dependencies operator="And">
                                                <fileDependency file="piggypet.esp" state="Active"/>
                                            </dependencies>
                                            <type name="Recommended"/>
                                        </pattern>
                                    </patterns>
                                </dependencyType>
                            </typeDescriptor>
                        </plugin>
                    </plugins>
                </group>
            </optionalFileGroups>
        </installStep>
    </installSteps>
</config>
"""


def info(version: str) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<fomod>
    <Name>Cat Carry</Name>
    <Author>W1ter</Author>
    <Version>{version}</Version>
    <Description>Crimson Desert style: hold Activate to pick up house cats (and Piglet) and carry them in your arms. Requires {HOUSE_CATS}.</Description>
    <Groups>
        <element>Animals</element>
    </Groups>
</fomod>
"""


def folder_files(folder: Path, entry: str) -> dict[str, Path]:
    return {f"{entry}/" + p.relative_to(folder).as_posix(): p for p in folder.rglob("*") if p.is_file()}


text = (ROOT / "xmake.lua").read_text(encoding="utf-8")
version = re.search(r'target\("CatCarry"\)\s*\n\s*set_version\("([^"]+)"\)', text).group(1)
version = re.sub(r"\.0$", "", version)
dll = next(BUILD.rglob("releasedbg/CatCarry.dll"))

files = {
    "00 Core/SKSE/Plugins/CatCarry.dll": dll,
    "00 Core/SKSE/Plugins/CatCarry.pdb": dll.with_suffix(".pdb"),
}
files |= folder_files(RES / "core", "00 Core")
files |= folder_files(RES / "01 House Cats", "01 House Cats")
files |= folder_files(RES / "10 Piglet", "10 Piglet")

OUTPUT.mkdir(parents=True, exist_ok=True)
archive = OUTPUT / f"Cat Carry {version}.zip"
with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
    for entry, path in files.items():
        z.write(path, entry)
    z.writestr("fomod/info.xml", info(version))
    z.writestr("fomod/ModuleConfig.xml", MODULE_CONFIG)
print(archive)
