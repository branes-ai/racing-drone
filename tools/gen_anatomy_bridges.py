#!/usr/bin/env python3
"""Generate racing-drone's anatomy bridge headers (docs/plan/consume-anatomy/).

    tools/gen_anatomy_bridges.py --build <build-dir>

The framework lives in anatomy (docs/adr/0008-anatomy-mission-framework.md).
Racing code names its types as branes::racing::X, so each bridge header brings
every namespace-scope name of one anatomy library into the matching
branes::racing namespace by an explicit using-declaration:

    sdk/include/branes/racing/anatomy.hpp           branes::anatomy (core)
    sdk/include/branes/racing/anatomy_mobility.hpp  branes::anatomy_mobility
    sdk/include/branes/racing/anatomy_glue.hpp      branes::anatomy_glue
    sdk/include/branes/racing/anatomy_sim.hpp       branes::anatomy_mobility_sim
    sdk/include/branes/racing/anatomy_sweep.hpp     the precision-sweep types

The names come from Clang's AST of anatomy's headers, at the anatomy commit the
build fetched, so the lists are complete and exact. Rerun it after bumping the
anatomy pin, from the source root, with a configured build directory (for
anatomy's and its dependencies' sources), and CLANG_FORMAT naming CI's
clang-format (18); then review the diff.
"""

import argparse
import collections
import json
import os
import subprocess
import sys
import tempfile

KINDS = {
    "CXXRecordDecl", "ClassTemplateDecl", "FunctionDecl", "FunctionTemplateDecl", "VarDecl",
    "TypeAliasDecl", "TypedefDecl", "ConceptDecl", "EnumDecl", "TypeAliasTemplateDecl",
    "VarTemplateDecl",
}

BRIDGES = {
    "core": ("anatomy.hpp", "branes::anatomy (the core: time, geometry, camera, profile, slots, baseline, log, harness)", []),
    "mobility": ("anatomy_mobility.hpp", "branes::anatomy_mobility (a roaming vehicle's contracts, configs and slots)",
                 ["branes/racing/anatomy.hpp"]),
    "glue": ("anatomy_glue.hpp", "branes::anatomy_glue (the reflex adapters)", ["branes/racing/anatomy_mobility.hpp"]),
    "sim": ("anatomy_sim.hpp", "branes::anatomy_mobility_sim (the closed-loop simulator)", ["branes/racing/anatomy_glue.hpp"]),
    "sweep": ("anatomy_sweep.hpp", "the precision-sweep types of branes::anatomy_mobility_sim (needs Universal)", []),
}


def library_of(path):
    path = os.path.realpath(path)
    if "/sdk/include/" in path:
        return "core"
    if "/mobility/include/" in path:
        return "mobility"
    if "/glue/include/" in path:
        return "glue"
    if path.endswith("/sim/sweep_types.hpp"):
        return "sweep"
    if "/sim/include/" in path:
        return "sim"
    raise SystemExit(f"gen_anatomy_bridges: {path} is in no anatomy library")


def ast_objects(text):
    decoder = json.JSONDecoder()
    i = 0
    while True:
        j = text.find("{", i)
        if j < 0:
            return
        try:
            obj, end = decoder.raw_decode(text, j)
        except json.JSONDecodeError:
            i = j + 1
            continue
        yield obj
        i = end


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", required=True, help="a configured build directory (its _deps/ holds the sources)")
    ap.add_argument("--out", default="sdk/include/branes/racing", help="where the bridge headers go")
    args = ap.parse_args()

    deps = os.path.join(args.build, "_deps")
    anatomy = os.path.join(deps, "anatomy-src")
    roots = [os.path.join(anatomy, p, "include") for p in ("sdk", "mobility", "glue", "sim")]
    headers = []
    for root in roots:
        found = []
        for dirpath, _, files in os.walk(root):
            found += [os.path.relpath(os.path.join(dirpath, f), root) for f in files if f.endswith(".hpp")]
        # A build without anatomy's sources would otherwise empty every bridge.
        if not found:
            raise SystemExit(f"gen_anatomy_bridges: no headers under {root}; is {args.build} configured?")
        headers += found
    includes = [f"-I{r}" for r in roots] + [
        f"-isystem{os.path.join(deps, p)}"
        for p in ("reflex-src/sdk/include", "mtl5-src/include", "mtl5-build/include", "universal-src/include",
                  "universal-src/include/sw")
    ]
    with tempfile.NamedTemporaryFile("w", suffix=".hpp", delete=False) as tu:
        tu.write("".join(f"#include <{h}>\n" for h in sorted(headers)))
    try:
        dump = subprocess.run(
            ["clang++", "-std=c++20", "-fsyntax-only", "-Xclang", "-ast-dump=json", "-Xclang",
             "-ast-dump-filter=branes", *includes, "-x", "c++", tu.name],
            check=True, capture_output=True, text=True).stdout
    finally:
        os.unlink(tu.name)

    names = collections.defaultdict(lambda: collections.defaultdict(set))  # library -> namespace -> names
    unscoped = set()  # (namespace, name) of unscoped enums: their enumerators come with `using enum`
    aliases = collections.defaultdict(lambda: collections.defaultdict(set))  # namespace aliases
    files = collections.defaultdict(set)  # library -> headers
    last_file = [None]

    def walk(node, ns):
        loc = node.get("loc", {})
        f = loc.get("file") or loc.get("spellingLoc", {}).get("file") or loc.get("expansionLoc", {}).get("file")
        if f:
            last_file[0] = f
        if node.get("kind") == "NamespaceDecl":
            for child in node.get("inner", []):
                walk(child, ns + [node.get("name", "")])
            return
        if not (node.get("name") and not node.get("isImplicit") and ns[:2] == ["branes", "anatomy"]):
            return
        if node.get("kind") == "NamespaceAliasDecl":
            lib = library_of(last_file[0])
            aliases[lib]["::".join(ns)].add(node["name"])
            files[lib].add(os.path.realpath(last_file[0]).split("/include/", 1)[1])
        elif node.get("kind") in KINDS:
            lib = library_of(last_file[0])
            names[lib]["::".join(ns)].add(node["name"])
            files[lib].add(os.path.realpath(last_file[0]).split("/include/", 1)[1])
            if node.get("kind") == "EnumDecl" and not node.get("scopedEnumTag"):
                unscoped.add(("::".join(ns), node["name"]))

    for obj in ast_objects(dump):
        if obj.get("kind") == "NamespaceDecl":
            walk(obj, [])

    # Write nothing unless every bridge has names: a partial AST must not
    # replace a good header with an empty one.
    for lib, (file_name, _, _) in BRIDGES.items():
        if not names[lib]:
            raise SystemExit(f"gen_anatomy_bridges: no names for {file_name}; nothing written")

    for lib, (file_name, what, extra) in BRIDGES.items():
        out = [
            "#pragma once",
            f"/// @file {file_name}",
            f"/// @brief The names racing-drone uses from {what}.",
            "///",
            "/// GENERATED by tools/gen_anatomy_bridges.py from anatomy's headers. Do not",
            "/// edit; rerun it after bumping the anatomy pin (docs/plan/consume-anatomy/).",
            "///",
            "/// The framework lives in anatomy (docs/adr/0008-anatomy-mission-framework.md).",
            "/// Each name is brought into the matching branes::racing namespace by an",
            "/// explicit using-declaration, so racing code names it as before; there is no",
            "/// using-directive.",
            "",
        ]
        out += [f"#include <{h}>" for h in extra]
        out += [f"#include <{h}>" for h in sorted(files[lib])]
        out.append("")
        for ns in sorted(set(names[lib]) | set(aliases[lib])):
            racing_ns = ns.replace("branes::anatomy", "branes::racing", 1)
            short = ns.removeprefix("branes::")
            out += [f"namespace {racing_ns} {{", ""]
            out += [f"namespace {a} = {short}::{a};" for a in sorted(aliases[lib][ns])]
            for n in sorted(names[lib][ns]):
                # An unscoped enum's enumerators are names of the namespace too.
                out.append(f"using {short}::{n};")
                if (ns, n) in unscoped:
                    out.append(f"using enum {short}::{n};")
            out += ["", f"}}  // namespace {racing_ns}", ""]
        path = os.path.join(args.out, file_name)
        with open(path, "w") as f:
            f.write("\n".join(out).rstrip("\n") + "\n")
        # clang-format orders the using-declarations and includes its own way;
        # use the version CI lints with (18), e.g. CLANG_FORMAT=clang-format-18.
        subprocess.run([os.environ.get("CLANG_FORMAT", "clang-format"), "-i", path], check=True)
        count = sum(len(v) for v in names[lib].values())
        print(f"gen_anatomy_bridges: {path}: {count} names from {len(files[lib])} headers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
