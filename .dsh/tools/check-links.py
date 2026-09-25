#!/usr/bin/env python3
"""Check that every relative Markdown link and anchor in the corpus resolves.

Run:  python3 .dsh/tools/check-links.py [root ...]

Default roots: the project directory and the user-level skill source. Fenced code
blocks are stripped first, so template examples inside SKILL.md references are not
mistaken for real links.
"""
from __future__ import annotations

import re
import sys
import unicodedata
from pathlib import Path
from urllib.parse import unquote

FENCE = re.compile(r"^(\s*)(```|~~~)")
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
HEADING = re.compile(r"^(#{1,6})\s+(.+?)\s*$")
# GitHub strips these from heading anchors.
KEEP_PUNCT = {"-", "_"}  # Pd and Pc survive GitHub's anchor slugging


def strip_fences(text: str) -> str:
    out, in_fence = [], False
    for line in text.splitlines():
        if FENCE.match(line):
            in_fence = not in_fence
            out.append("")
            continue
        out.append("" if in_fence else line)
    return "\n".join(out)


def slugify(heading: str) -> str:
    """Approximate GitHub's heading anchor: keep letters, numbers, marks, '-' and '_'."""
    text = heading.strip().lower()
    text = re.sub(r"`([^`]*)`", r"\1", text)
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = unicodedata.normalize("NFKC", text)
    kept = []
    for ch in text:
        category = unicodedata.category(ch)[0]
        if category in ("L", "N", "M") or ch in KEEP_PUNCT:
            kept.append(ch)
        elif ch.isspace():
            kept.append(" ")
    return re.sub(r"\s+", "-", "".join(kept)).strip("-")


def headings(path: Path) -> set[str]:
    text = strip_fences(path.read_text(encoding="utf-8", errors="replace"))
    found = set()
    for line in text.splitlines():
        m = HEADING.match(line)
        if m:
            found.add(slugify(m.group(2)))
    return found


def main(argv: list[str]) -> int:
    roots = [Path(a) for a in argv[1:]] or [Path.cwd()]
    files: list[Path] = []
    for root in roots:
        files.extend(sorted(p for p in root.rglob("*.md") if p.is_file()))

    heading_cache: dict[Path, set[str]] = {}
    errors: list[str] = []
    checked = 0

    for src in files:
        text = strip_fences(src.read_text(encoding="utf-8", errors="replace"))
        for target in LINK.findall(text):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            checked += 1
            target_path = unquote(target)
            file_part, _, anchor = target_path.partition("#")
            if not file_part:  # same-file anchor
                dest = src
            else:
                dest = (src.parent / file_part).resolve()
            if not dest.exists():
                errors.append(f"{src}: missing file -> {target}")
                continue
            if anchor:
                if dest not in heading_cache:
                    heading_cache[dest] = headings(dest)
                if anchor not in heading_cache[dest]:
                    errors.append(f"{src}: missing anchor -> {target}")

    print(f"scanned {len(files)} markdown files, checked {checked} relative links")
    if errors:
        print(f"\n{len(errors)} problem(s):")
        for e in errors:
            print("  FAIL " + e)
        return 1
    print("all relative links and anchors resolve")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
