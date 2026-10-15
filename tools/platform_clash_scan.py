#!/usr/bin/env python3
"""Compile every translation unit with another platform's global names forced in, to find OPAD names that clash with them.

Platform headers put names at global scope: X11's `typedef struct _XDisplay Display` (Qt's qguiapplication_platform.h on
X11), windows.h's GDI functions (`Polygon`, `Rectangle`, `Ellipse`, `Arc`, ...) and its macros (`DrawText`, `GetObject`,
`ERROR`, `TRANSPARENT`, `near`, `far`, ...). Which files see them depends on the platform and on how deep Qt and OCCT
include them, which changes between versions: a file that builds today can break on the next Qt. This replays each
command of `compile_commands.json` with `-fsyntax-only` and the platform's names force-included, so any file that would
clash fails here, on any machine.

  python3 tools/platform_clash_scan.py build/linux --x11              # QtGui files with X11's names (Qt's own header)
  python3 tools/platform_clash_scan.py build/linux --windows <dir>    # every file with windows.h's names; <dir> holds
                                                                       # windows-names.h (see --make-windows-names)
  python3 tools/platform_clash_scan.py --macros <dir>                  # OPAD identifiers that windows.h defines as macros
  python3 tools/platform_clash_scan.py build/linux --reach <file>...   # does windows.h reach them on Windows, and how
  python3 tools/platform_clash_scan.py --make-windows-names <dir> --cross x86_64-w64-mingw32-g++
      writes <dir>/windows-names.h: the macros and global functions of MinGW's windows.h as OPAD's Windows build includes
      it (NOMINMAX and WIN32_LEAN_AND_MEAN, CMakeLists.txt), in a form the host compiler takes, so the clash can be
      checked without a Windows build (the cross compiler only reads the headers)

Exit status 1 when a file fails; each failure is listed with its first errors.
"""
import argparse
import concurrent.futures
import json
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def commands(build: Path, only_qtgui: bool):
    for entry in json.loads((build / "compile_commands.json").read_text()):
        args = entry.get("arguments") or shlex.split(entry["command"])
        source = Path(entry["file"])
        if only_qtgui and not any("QtGui" in a for a in args):
            continue
        rel = source.resolve().relative_to(ROOT) if source.resolve().is_relative_to(ROOT) else source
        if str(rel).startswith("build/"):  # generated (moc, rcc): Qt's own code
            continue
        out, skip = [], 0
        for a in args:  # no object file, no dependency file: nothing is written
            if skip:
                skip -= 1
                continue
            if a in ("-o", "-MF", "-MT", "-MQ"):
                skip = 1
                continue
            if a in ("-MD", "-MMD", "-c"):
                continue
            out.append(a)
        yield str(rel), out, entry.get("directory", str(build))


def check(item, forced):
    rel, args, cwd = item
    cmd = args[:1] + ["-fsyntax-only", "-w"] + [x for f in forced for x in ("-include", f)] + args[1:]
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    errors = [l for l in r.stderr.splitlines() if " error: " in l or l.startswith("error:")]
    return rel, r.returncode, errors


# MinGW's windows.h as OPAD's Windows build sees it, read by the cross compiler and handed on in a form a host compiler
# takes: the names the Windows API headers put at global scope (not those of the C runtime under them, which the host has
# its own of). Macros, with the helper macros they expand to; API functions and typedefs as declarations of their names
# alone (only the name matters for a clash).
CRT = re.compile(r"(^|/)(_mingw[^/]*|errno|limits|stdlib|ctype|stdarg|string|stddef|malloc|excpt|wchar|stdio|vadefs|crtdefs|corecrt[^/]*)\.h$|"
                 r"/sdks/|/sec_api/|/lib/gcc/|<built-in>|<command-line>|<stdin>")


def make_windows_names(out_dir: Path, cross: str):
    out_dir.mkdir(parents=True, exist_ok=True)
    src = '#define NOMINMAX\n#define WIN32_LEAN_AND_MEAN\n#include <windows.h>\n'  # as CMakeLists.txt defines them on Windows
    text = subprocess.run([cross, "-x", "c++", "-E", "-dD", "-"], input=src, capture_output=True, text=True, check=True).stdout
    defines, code, where = {}, [], "?"
    for line in text.splitlines():
        if m := re.match(r'# \d+ "([^"]+)"', line):
            where = m.group(1)
            continue
        if line.startswith("#define "):
            name = line.split()[1].split("(")[0]
            defines[name] = (line, where)
        elif line.startswith("#undef "):
            defines.pop(line.split()[1], None)
        elif not CRT.search(where):
            code.append(line)
    # not the compiler's intrinsics (a host compiler has its own, under the same names)
    keep = {n for n, (line, where) in defines.items() if not n.startswith("_") and not CRT.search(where) and "__builtin" not in line}
    todo = list(keep)
    while todo:  # the helper macros the kept ones expand to (__MINGW_NAME_AW, __MSABI_LONG, ...)
        body = defines[todo.pop()][0]
        for word in set(re.findall(r"\b_\w+", body)):
            if word in defines and word not in keep and not defines[word][1].startswith("<built-in>") and "__builtin" not in defines[word][0]:
                keep.add(word)
                todo.append(word)
    body = "\n".join(code).replace("__attribute__((dllimport))", " IMPORTED ")
    body = re.sub(r"__attribute__\s*\(\((?:[^()]|\([^()]*\))*\)\)", " ", body)  # __stdcall__ and the like
    funcs, types = set(), set()
    # top-level statements only (a struct's members are not global names): split at ';' outside braces
    depth, start, braces = 0, 0, []  # extern "C" { ... } blocks do not count
    for i, ch in enumerate(body):
        if ch == "{":
            transparent = body[start:i].strip().endswith('extern "C"')
            braces.append(transparent)
            if transparent:
                start = i + 1
            else:
                depth += 1
        elif ch == "}":
            if braces and braces.pop():
                start = i + 1
            else:
                depth -= 1
        elif ch == ";" and depth == 0:
            stmt = " ".join(body[start:i].split())
            start = i + 1
            if m := re.match(r"(?:extern\s+\"C\"\s*\{\s*)?IMPORTED\b[^(){}]*?\b([A-Za-z]\w*)\s*\(", stmt):
                funcs.add(m.group(1))
            elif stmt.startswith("typedef "):
                tail = stmt[stmt.rfind("}") + 1:] if "}" in stmt else stmt[len("typedef "):]
                if m := re.search(r"\(\s*\*\s*([A-Za-z]\w*)\s*\)\s*\(", tail):  # a function pointer type
                    types.add(m.group(1))
                elif "(" not in tail:  # the declarators: the last word of each (NAME, *PNAME, NAME[4])
                    for part in tail.split(","):
                        if m := re.search(r"([A-Za-z]\w*)\s*(?:\[[^\]]*\])*\s*$", part.strip()):
                            types.add(m.group(1))
    types -= funcs
    for n in list(funcs | types):  # a macro of the same name stands for it
        if n in keep:
            funcs.discard(n), types.discard(n)
    lines = [defines[n][0] for n in sorted(keep, key=lambda n: (not n.startswith("_"), n))]
    lines += [f"struct opad_windows_{t}; typedef opad_windows_{t} {t};" for t in sorted(types)]
    lines += [f"int {f}(...);" for f in sorted(funcs)]
    (out_dir / "windows-names.h").write_text(
        "// Generated by tools/platform_clash_scan.py --make-windows-names from MinGW's windows.h (NOMINMAX, WIN32_LEAN_AND_MEAN):\n"
        "// the macros, typedefs and API functions it puts at global scope, for a host compiler. Not a usable windows.h.\n"
        "#pragma once\n" + "\n".join(lines) + "\n")
    print(f"{out_dir / 'windows-names.h'}: {len(keep)} macros, {len(types)} types, {len(funcs)} functions")


# OPAD's identifiers that windows.h defines as macros. Inside one file a macro renames the declaration and every use alike,
# so the compile check passes; it breaks when a file that sees windows.h and one that does not disagree on a member's
# name (DrawText -> DrawTextW in one, DrawText in the other: an undefined reference at link time, or "no member").
def macro_report(names_dir: Path):
    header = (names_dir / "windows-names.h").read_text()
    macros = {m.group(1) for m in re.finditer(r"^#define ([A-Za-z]\w*)", header, re.M)}
    strip = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)
    hits = {}
    for folder in ("app", "core", "cli", "paint", "plugins", "shell", "tests"):
        for f in sorted((ROOT / folder).rglob("*")):
            if f.suffix not in (".cpp", ".hpp", ".h", ".cc", ".inc") or "third_party" in f.parts:
                continue
            text = strip.sub(lambda m: "\n" * m.group(0).count("\n"), f.read_text(errors="replace"))
            for n, line in enumerate(text.splitlines(), 1):
                for m in re.finditer(r"\b[A-Za-z]\w*\b", line):
                    word = m.group(0)
                    if word in macros:
                        before = line[: m.start()].rstrip()
                        member = before.endswith((".", "->", "::")) or re.match(r"\s*\(", line[m.end():]) and re.search(r"\b(void|bool|int|double|auto|QString|std::\w+)\s*$", before)
                        hits.setdefault(word, []).append((f.relative_to(ROOT), n, bool(member), line.strip()[:110]))
    risky = 0
    for word in sorted(hits, key=lambda w: (not any(h[2] for h in hits[w]), w)):
        uses = hits[word]
        flag = any(h[2] for h in uses)
        risky += flag
        print(f"{'MEMBER ' if flag else '       '}{word}: {len(uses)} uses in {len({h[0] for h in uses})} files")
        for f, n, member, text in uses[:3]:
            print(f"         {f}:{n}  {text}")
    print(f"{len(hits)} identifiers are windows.h macros, {risky} of them used as a member or declared as a function")
    return 1 if risky else 0


# Whether windows.h really reaches a file when it is built for Windows: the file preprocessed by the cross compiler with
# its own include paths (the host's Qt and OCCT headers, which take their _WIN32 branches), and the chain of headers that
# brought it in. Qt and OCCT on the Windows machine may be other versions than the host's: an approximation.
def reach(build: Path, files, cross: str):
    wanted = {str(Path(f)) for f in files}
    found = 0
    for rel, args, cwd in commands(build, False):
        if rel not in wanted:
            continue
        fixed, skip = [], False
        for x in args[1:]:  # keep -isystem <dir> pairs whole
            if skip:
                fixed.append(x)
                skip = False
            elif x == "-isystem":
                fixed.append(x)
                skip = True
            elif x.startswith(("-I", "-D", "-std=", "-isystem")):
                fixed.append(x)
        cmd = [cross, "-E", "-H", "-o", os.devnull, "-DNOMINMAX", "-DWIN32_LEAN_AND_MEAN", "-DUNICODE", "-D_UNICODE"] + fixed + [str(ROOT / rel)]
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
        chain, hit = [], None
        for line in r.stderr.splitlines():
            m = re.match(r"(\.+) (\S+)", line)
            if not m:
                continue
            depth = len(m.group(1))
            chain = chain[: depth - 1] + [m.group(2)]
            if os.path.basename(m.group(2)).lower() == "windows.h":
                hit = list(chain)
                break
        if hit:
            found += 1
            short = [h.replace(str(ROOT) + "/", "").split("/include/")[-1] for h in hit]
            print(f"REACHES {rel}: " + " -> ".join(short))
        else:
            print(f"no      {rel}")
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build", nargs="?", type=Path)
    ap.add_argument("--x11", action="store_true", help="force Qt's QGuiApplication (X11's Display) into QtGui files")
    ap.add_argument("--windows", type=Path, metavar="DIR", help="force DIR/windows-names.h into every file")
    ap.add_argument("--make-windows-names", type=Path, metavar="DIR")
    ap.add_argument("--macros", type=Path, metavar="DIR", help="list OPAD identifiers that DIR/windows-names.h defines as macros")
    ap.add_argument("--reach", nargs="+", metavar="FILE", help="whether windows.h reaches these files when built for Windows (the build folder's flags, the cross compiler)")
    ap.add_argument("--cross", default="x86_64-w64-mingw32-g++")
    ap.add_argument("-j", type=int, default=os.cpu_count())
    a = ap.parse_args()
    if a.make_windows_names:
        return make_windows_names(a.make_windows_names, a.cross)
    if a.macros:
        return macro_report(a.macros)
    if a.reach:
        if not a.build:
            ap.error("--reach needs the build folder")
        reach(a.build, a.reach, a.cross)
        return 0
    if not a.build or a.x11 == bool(a.windows):
        ap.error("a build folder and one of --x11 / --windows")
    forced = ["QtGui/QGuiApplication"] if a.x11 else [str((a.windows / "windows-names.h").resolve())]
    items = list(commands(a.build, a.x11))
    failed = 0
    with concurrent.futures.ThreadPoolExecutor(a.j) as pool:
        for rel, code, errors in pool.map(lambda i: check(i, forced), items):
            if code:
                failed += 1
                print(f"FAIL {rel}")
                for e in errors[:4]:
                    print("     " + e)
    print(f"{len(items)} files, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
