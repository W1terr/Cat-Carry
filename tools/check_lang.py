"""
Checks the translations in res/core/SKSE/Plugins/CatCarry/lang against the texts the code translates.

The English texts are the keys: T("..."), F("..."), Id("..."), Hint("..."), Lang::Copy("..."), kGlyphHint and the animal
names (sName of the animal files, shown in the menu).
Reports keys missing from a language file, keys the code no longer uses and broken {} / % placeholders.
usage: python check_lang.py [--keys]   (--keys: print the keys as a json template)
"""

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
SOURCES = [ROOT / "src" / name for name in ("Menu.cpp", "Carry.cpp", "Hooks.cpp")]
LANG_DIR = ROOT / "res" / "core" / "SKSE" / "Plugins" / "CatCarry" / "lang"
LANGUAGES = ["french", "german", "italian", "spanish", "polish", "russian", "japanese", "chinese"]

LITERAL = r'"(?:[^"\\]|\\.)*"'


def code_keys() -> list[str]:
    keys = []

    def add(key):
        if key not in keys:
            keys.append(key)

    for path in SOURCES:
        src = re.sub(r"//[^\n]*", "", path.read_text(encoding="utf-8"))
        for m in re.finditer(rf"\b(?:T|F|Id|Hint|Lang::Copy)\(\s*({LITERAL})", src):
            add(json.loads(m.group(1)))
        for m in re.finditer(rf"kGlyphHint\s*=\s*({LITERAL})", src):
            add(json.loads(m.group(1)))
    for path in sorted((ROOT / "res").rglob("Animals/*.ini")):
        for line in path.read_text(encoding="utf-8").splitlines():
            m = re.match(r"\s*sName\s*=\s*(.+?)\s*$", line)
            if m:
                add(m.group(1))
    return keys


def placeholders(text: str) -> list[str]:
    return sorted(re.findall(r"\{[^}]*\}|%[-+ #0-9.]*[a-zA-Z]", text))


def main():
    keys = code_keys()
    if "--keys" in sys.argv:
        print(json.dumps({k: "" for k in keys}, ensure_ascii=False, indent="\t"))
        return
    ok = True
    for language in LANGUAGES:
        path = LANG_DIR / f"{language}.json"
        if not path.exists():
            print(f"{language}: MISSING FILE")
            ok = False
            continue
        table = json.loads(path.read_text(encoding="utf-8"))
        missing = [k for k in keys if not table.get(k)]
        unused = [k for k in table if k not in keys]
        broken = [k for k in keys if table.get(k) and placeholders(table[k]) != placeholders(k)]
        for title, items in (("missing", missing), ("unused", unused), ("broken placeholders", broken)):
            if items:
                ok = False
                print(f"{language}: {len(items)} {title}:")
                for item in items:
                    print(f"    {item}")
        print(f"{language}: {len(keys) - len(missing)}/{len(keys)}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
