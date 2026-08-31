#!/usr/bin/env python3
"""Build a reproducible callable-API inventory for every reached Jolt header.

The overload-count comparison is intentionally a candidate generator. C++ and
Nim spell types and default arguments differently, so every reported deficit
still needs a focused compile test before it is considered a binding gap.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


CALLABLE_KINDS = {
    "CXXConstructorDecl",
    "CXXMethodDecl",
    "CXXConversionDecl",
    "FunctionDecl",
}
IGNORED_OPERATORS = {
    "operator new",
    "operator new[]",
    "operator delete",
    "operator delete[]",
}
OPERATOR_NAMES = {
    "operator==": "==",
    "operator!=": "!=",
    "operator<": "<",
    "operator<=": "<=",
    "operator>": ">",
    "operator>=": ">=",
    "operator+": "+",
    "operator-": "-",
    "operator*": "*",
    "operator/": "/",
    "operator[]": "[]",
}
RAW_WRAPPER_NAMES = {
    "addAssign": "operator+=",
    "advance": "operator++",
    "apply": "operator()",
    "bitAnd": "operator&",
    "bitAndAssign": "operator&=",
    "bitNot": "operator~",
    "bitOr": "operator|",
    "bitOrAssign": "operator|=",
    "bitXor": "operator^",
    "bitXorAssign": "operator^=",
    "divAssign": "operator/=",
    "fromFloatFallbackMode": "FromFloatFallback",
    "fromFloatMode": "FromFloat",
    "hlslAdd": "+",
    "hlslAtomicAdd": "JPH_AtomicAdd",
    "hlslDiv": "/",
    "hlslDivScalar": "/",
    "hlslDot": "dot",
    "hlslDotFloat2": "dot",
    "hlslDotFloat3": "dot",
    "hlslDotFloat4": "dot",
    "hlslDotInt3": "dot",
    "hlslDotInt4": "dot",
    "hlslDotUint3": "dot",
    "hlslDotUint4": "dot",
    "hlslIndexMutable": "[]",
    "hlslLength": "length",
    "hlslMax": "max",
    "hlslMin": "min",
    "hlslMul": "*",
    "hlslMulLeft": "*",
    "hlslMulRight": "*",
    "hlslNormalize": "normalize",
    "hlslRound": "round",
    "hlslSub": "-",
    "realLiteral": "operator\"\"_r",
    "fromFloatFallbackNegInf": "FromFloatFallback",
    "fromFloatFallbackNearest": "FromFloatFallback",
    "fromFloatFallbackPosInf": "FromFloatFallback",
    "fromFloatNegInf": "FromFloat",
    "fromFloatNearest": "FromFloat",
    "fromFloatPosInf": "FromFloat",
    "mulAssign": "operator*=",
    "scaleLeft": "*",
    "subAssign": "operator-=",
    "toBroadPhaseLayerType": "operator unsigned char",
    "toRayCast": "operator RayCast",
    "toShapeCast": "operator ShapeCast",
    "toVec3": "operator Vec3",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("jolt_source", type=Path)
    parser.add_argument(
        "--raw-api",
        type=Path,
        default=Path("src/jolt/raw_api.nim"),
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--clang", default="clang++")
    parser.add_argument(
        "--debug-renderer",
        action="store_true",
        help="inventory the JPH_DEBUG_RENDERER configuration",
    )
    parser.add_argument(
        "--compute-backend",
        choices=("cpu", "vk", "dx12", "mtl"),
        default="cpu",
        help="select the Jolt compute backend configuration (default: cpu)",
    )
    parser.add_argument(
        "--include-dir",
        action="append",
        type=Path,
        default=[],
        help="add an SDK include directory required by the selected backend",
    )
    return parser.parse_args()


def node_file(node: dict[str, Any], inherited: str = "") -> str:
    loc = node.get("loc", {})
    direct = loc.get("file")
    if direct:
        return direct
    begin = node.get("range", {}).get("begin", {})
    return begin.get("file", inherited)


def jolt_header(path: str) -> str | None:
    marker = "/Jolt/"
    if marker not in path:
        return None
    return "Jolt/" + path.split(marker, 1)[1]


def normalized_callable_name(kind: str, name: str) -> str:
    if kind == "CXXConstructorDecl":
        return "<ctor>"
    return OPERATOR_NAMES.get(name, name)


def make_entry(
    node: dict[str, Any],
    owner: str,
    inherited_file: str,
    is_abstract: bool,
) -> dict[str, Any] | None:
    kind = node.get("kind", "")
    name = node.get("name", "")
    if kind not in CALLABLE_KINDS or not name:
        return None
    if node.get("isImplicit") or node.get("explicitlyDeleted"):
        return None
    if name in IGNORED_OPERATORS or kind == "CXXDestructorDecl":
        return None
    if kind == "CXXConstructorDecl" and is_abstract:
        return None
    source = node_file(node, inherited_file)
    header = jolt_header(source)
    if header is None:
        return None
    return {
        "owner": owner,
        "name": normalized_callable_name(kind, name),
        "signature": node.get("type", {}).get("qualType", ""),
        "kind": "constructor" if kind == "CXXConstructorDecl" else "callable",
        "header": header,
        "line": node.get("loc", {}).get("line", 0),
    }


def template_callable(node: dict[str, Any]) -> dict[str, Any] | None:
    for child in node.get("inner", []):
        if child.get("kind") in CALLABLE_KINDS:
            return child
    return None


def record_entries(
    record: dict[str, Any],
    parent_owner: str = "JPH",
    inherited_file: str = "",
) -> Iterable[dict[str, Any]]:
    if not record.get("completeDefinition"):
        return
    name = record.get("name", "")
    if not name or record.get("isImplicit"):
        return
    owner = f"{parent_owner}::{name}"
    source = node_file(record, inherited_file)
    default_access = "public" if record.get("tagUsed") in {"struct", "union"} else "private"
    access = default_access
    is_abstract = record.get("definitionData", {}).get("isAbstract", False)

    for child in record.get("inner", []):
        kind = child.get("kind", "")
        if kind == "AccessSpecDecl":
            access = child.get("access", access)
            continue
        child_access = child.get("access", access)
        if kind in {"CXXRecordDecl", "ClassTemplateDecl"}:
            if child_access != "public":
                continue
            nested = child
            if kind == "ClassTemplateDecl":
                nested = next(
                    (
                        item
                        for item in child.get("inner", [])
                        if item.get("kind") == "CXXRecordDecl"
                    ),
                    {},
                )
            yield from record_entries(nested, owner, source)
            continue
        if child_access != "public":
            continue
        callable_node = child
        if kind == "FunctionTemplateDecl":
            callable_node = template_callable(child) or {}
        entry = make_entry(callable_node, owner, source, is_abstract)
        if entry is not None:
            yield entry


def namespace_entries(
    namespace: dict[str, Any],
    parent_owner: str = "JPH",
    inherited_file: str = "",
) -> Iterable[dict[str, Any]]:
    """Inventory declarations nested below a namespace AST root.

    Clang's filtered JSON often emits nested namespaces such as
    JPH::HLSLToCPP as roots, without their semantic JPH parent.
    """
    name = namespace.get("name", "")
    owner = parent_owner if not name or name == "JPH" else f"{parent_owner}::{name}"
    source = node_file(namespace, inherited_file)
    for child in namespace.get("inner", []):
        kind = child.get("kind", "")
        if kind == "NamespaceDecl":
            yield from namespace_entries(child, owner, source)
        elif kind == "CXXRecordDecl":
            yield from record_entries(child, owner, source)
        elif kind == "ClassTemplateDecl":
            record = next(
                (
                    item
                    for item in child.get("inner", [])
                    if item.get("kind") == "CXXRecordDecl"
                ),
                {},
            )
            yield from record_entries(record, owner, source)
        elif kind in {"FunctionDecl", "FunctionTemplateDecl"}:
            callable_node = child if kind == "FunctionDecl" else template_callable(child)
            if callable_node is not None:
                entry = make_entry(callable_node, owner, source, False)
                if entry is not None:
                    yield entry


def load_concatenated_json(path: Path) -> Iterable[dict[str, Any]]:
    text = path.read_text(encoding="utf-8")
    decoder = json.JSONDecoder()
    position = 0
    while position < len(text):
        while position < len(text) and text[position].isspace():
            position += 1
        if position >= len(text):
            break
        value, position = decoder.raw_decode(text, position)
        yield value


def clang_inventory(
    clang: str,
    jolt_source: Path,
    umbrella: Path,
    debug_renderer: bool = False,
    compute_backend: str = "cpu",
    include_dirs: Iterable[Path] = (),
) -> list[dict[str, Any]]:
    if shutil.which(clang) is None:
        raise SystemExit(f"clang executable not found: {clang}")
    if not (jolt_source / "Jolt" / "Jolt.h").is_file():
        raise SystemExit("jolt_source must contain Jolt/Jolt.h")
    with tempfile.TemporaryDirectory(prefix="jolt-nim-api-audit-") as temp_dir:
        ast_path = Path(temp_dir) / "jolt-physics-ast.json"
        command = [
            clang,
            "-std=c++17",
            f"-I{jolt_source}",
            "-DNDEBUG",
            "-DJPH_OBJECT_STREAM",
            "-Xclang",
            "-ast-dump=json",
            "-Xclang",
            "-ast-dump-filter=JPH::",
            "-fsyntax-only",
            "-x",
            "c++",
            str(umbrella),
        ]
        backend_define = {
            "cpu": "JPH_USE_CPU_COMPUTE",
            "vk": "JPH_USE_VK",
            "dx12": "JPH_USE_DX12",
            "mtl": "JPH_USE_MTL",
        }[compute_backend]
        command.insert(-4, f"-D{backend_define}")
        if compute_backend == "dx12":
            command.insert(-4, "-DJPH_PLATFORM_WINDOWS")
        elif compute_backend == "mtl":
            command.insert(-4, "-DJPH_PLATFORM_MACOS")
            command[-2] = "objective-c++"
        for include_dir in include_dirs:
            command.insert(-4, f"-I{include_dir.resolve()}")
        if debug_renderer:
            command.insert(-4, "-DJPH_DEBUG_RENDERER")
        with ast_path.open("w", encoding="utf-8") as output:
            subprocess.run(command, check=True, stdout=output)

        entries: list[dict[str, Any]] = []
        for root in load_concatenated_json(ast_path):
            kind = root.get("kind", "")
            if kind == "CXXRecordDecl":
                entries.extend(record_entries(root))
            elif kind == "ClassTemplateDecl":
                record = next(
                    (
                        item
                        for item in root.get("inner", [])
                        if item.get("kind") == "CXXRecordDecl"
                    ),
                    {},
                )
                entries.extend(record_entries(record))
            elif kind == "NamespaceDecl":
                entries.extend(namespace_entries(root))
            elif kind in {"FunctionDecl", "FunctionTemplateDecl"}:
                callable_node = root if kind == "FunctionDecl" else template_callable(root)
                # Clang's filtered AST also emits out-of-line method template
                # definitions as roots. Their owning class declaration was
                # already inventoried above, so treating them as JPH free
                # functions creates a duplicate with the wrong owner.
                if callable_node is not None and callable_node.get("kind") == "FunctionDecl":
                    entry = make_entry(callable_node, "JPH", node_file(root), False)
                    if entry is not None:
                        entries.append(entry)

    unique = {
        (
            entry["owner"],
            entry["name"],
            entry["signature"],
            entry["header"],
            entry["line"],
        ): entry
        for entry in entries
    }
    return sorted(
        unique.values(),
        key=lambda item: (
            item["header"],
            item["line"],
            item["owner"],
            item["name"],
            item["signature"],
        ),
    )


PROC_NAME_RE = re.compile(r"^\s*proc\s+(`[^`]+`|[A-Za-z_][A-Za-z0-9_]*)\*?")
SELF_RE = re.compile(r"(?:^|;)\s*self:\s*(?:var\s+)?(?:ptr\s+)?([A-Za-z_][A-Za-z0-9_]*)")
STATIC_RE = re.compile(r"(?:^|;)\s*_:\s*(?:type\s+|typedesc\[)([A-Za-z_][A-Za-z0-9_]*)")


def parse_proc_line(line: str) -> tuple[str, str, str | None] | None:
    name_match = PROC_NAME_RE.match(line)
    if name_match is None:
        return None
    open_paren = line.find("(", name_match.end())
    if open_paren < 0:
        return None
    depth = 0
    close_paren = -1
    for index in range(open_paren, len(line)):
        char = line[index]
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                close_paren = index
                break
    if close_paren < 0:
        return None
    parameters = line[open_paren + 1 : close_paren]
    suffix = line[close_paren + 1 :]
    result_match = re.match(r"\s*:\s*(.*?)\s*\{\.", suffix)
    result_type = result_match.group(1) if result_match else None
    return name_match.group(1), parameters, result_type


def raw_groups(
    raw_api: Path,
) -> tuple[Counter[tuple[str, str]], set[tuple[str, str]]]:
    groups: Counter[tuple[str, str]] = Counter()
    generic_groups: set[tuple[str, str]] = set()
    lines = raw_api.read_text(encoding="utf-8").splitlines()
    declarations: list[str] = []
    current: list[str] = []
    for line in lines:
        if PROC_NAME_RE.match(line):
            if current:
                declarations.append(" ".join(current))
            current = [line.strip()]
            if ".}" in line:
                declarations.append(" ".join(current))
                current = []
        elif current:
            current.append(line.strip())
            if ".}" in line:
                declarations.append(" ".join(current))
                current = []
    if current:
        declarations.append(" ".join(current))

    for declaration in declarations:
        parsed = parse_proc_line(declaration)
        if parsed is None:
            continue
        raw_name, parameters, result_type = parsed
        name = raw_name.strip("`")
        owner_match = SELF_RE.search(parameters) or STATIC_RE.search(parameters)
        if owner_match is not None:
            owner = owner_match.group(1)
            if owner == "JoltApi":
                # The generated low-level API uses JoltApi as a namespace
                # marker. Recover nested namespace ownership from importcpp
                # so JPH::ScaleHelpers::Foo and similar declarations are not
                # incorrectly attributed to the root JPH namespace.
                namespace_match = re.search(
                    r'importcpp:\s*"JPH::((?:[A-Za-z_][A-Za-z0-9_]*::)+)',
                    declaration,
                )
                owner = (
                    namespace_match.group(1).removesuffix("::").replace("::", "_")
                    if namespace_match is not None
                    else "JPH"
                )
        elif name.startswith("construct") and result_type:
            result_match = re.match(r"(?:ptr\s+)?([A-Za-z_][A-Za-z0-9_]*)", result_type)
            if result_match is None:
                continue
            owner = result_match.group(1)
            name = "<ctor>"
        else:
            owner = "JPH"
        if "operator=" in declaration:
            name = "operator="
        else:
            name = RAW_WRAPPER_NAMES.get(name, name)
        groups[(owner, name)] += 1
        name_match = PROC_NAME_RE.match(declaration)
        open_paren = declaration.find("(", name_match.end()) if name_match else -1
        if name_match and "[" in declaration[name_match.end() : open_paren]:
            generic_groups.add((owner, name))
    return groups, generic_groups


def without_cpp_const(signature: str) -> str:
    """Collapse C++ const-only overload distinctions that Nim cannot encode."""
    return re.sub(r"\s+", " ", re.sub(r"\bconst\s+", "", signature)).removesuffix(
        " const"
    )


def normalized_owner(owner: str) -> str:
    """Match C++ nested owners to the flattened public Nim type names."""
    if owner == "JPH":
        return owner
    return owner.removeprefix("JPH::").replace("::", "_")


def main() -> None:
    args = parse_args()
    raw_api = args.raw_api.resolve()
    project_root = raw_api.parents[2]
    umbrella = project_root / "src" / "jolt" / "private" / "jolt_api.hpp"
    entries = clang_inventory(
        args.clang,
        args.jolt_source.resolve(),
        umbrella,
        args.debug_renderer,
        args.compute_backend,
        args.include_dir,
    )
    upstream_groups = Counter(
        (normalized_owner(entry["owner"]), entry["name"])
        for entry in entries
    )
    upstream_group_entries: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for entry in entries:
        key = (normalized_owner(entry["owner"]), entry["name"])
        upstream_group_entries.setdefault(key, []).append(entry)
    nim_groups, nim_generic_groups = raw_groups(raw_api)
    deficits = []
    missing_groups = []
    const_collapsed_groups = []
    generic_overload_groups = []
    inaccessible_overload_groups = []
    unresolved_overload_groups = []
    for key, upstream_count in sorted(upstream_groups.items()):
        exact_count = nim_groups.get(key, 0)
        # Clang emits some out-of-line nested record definitions as namespace
        # roots and omits their semantic parent in the filtered JSON. Recover
        # the flattened Nim owner only when the suffix match is unambiguous.
        nested_matches = [
            count
            for (owner, name), count in nim_groups.items()
            if owner.endswith("_" + key[0]) and name == key[1]
        ]
        nim_count = (
            nested_matches[0]
            if exact_count == 0 and len(nested_matches) == 1
            else exact_count
        )
        if nim_count < upstream_count:
            candidate = {
                "owner": key[0],
                "name": key[1],
                "upstream_overloads": upstream_count,
                "nim_overloads": nim_count,
                "difference": upstream_count - nim_count,
            }
            deficits.append(candidate)
            if nim_count == 0:
                missing_groups.append(candidate)
            elif key in nim_generic_groups:
                candidate["classification"] = "nim-generic-overload-family"
                generic_overload_groups.append(candidate)
            elif (
                key == ("JobSystem_JobHandle", "<ctor>")
                or key == ("ObjectStreamIn", "ReadClassData")
            ):
                candidate["classification"] = "upstream-inaccessible-parameter"
                inaccessible_overload_groups.append(candidate)
            elif len(
                {
                    without_cpp_const(entry["signature"])
                    for entry in upstream_group_entries[key]
                }
            ) <= nim_count:
                candidate["classification"] = "const-collapsed"
                const_collapsed_groups.append(candidate)
            else:
                candidate["classification"] = "unresolved-overload-deficit"
                unresolved_overload_groups.append(candidate)

    result = {
        "scope": (
            "public callable declarations in every Jolt header reached by the binding umbrella"
            + (" with JPH_DEBUG_RENDERER" if args.debug_renderer else "")
            + f" using the {args.compute_backend} compute backend"
        ),
        "upstream_callable_signatures": len(entries),
        "upstream_owner_name_groups": len(upstream_groups),
        "nim_raw_procedure_declarations": sum(1 for line in raw_api.read_text(encoding="utf-8").splitlines() if re.match(r"^\s*proc\s+", line)),
        "overload_count_candidate_groups": len(deficits),
        "overload_count_candidate_difference": sum(item["difference"] for item in deficits),
        "missing_owner_name_groups": len(missing_groups),
        "missing_owner_name_difference": sum(item["difference"] for item in missing_groups),
        "const_collapsed_overload_groups": len(const_collapsed_groups),
        "nim_generic_overload_family_groups": len(generic_overload_groups),
        "upstream_inaccessible_overload_groups": len(inaccessible_overload_groups),
        "unresolved_overload_groups": len(unresolved_overload_groups),
        "callable_owner_name_coverage_percent": round(
            100.0 * (len(upstream_groups) - len(missing_groups)) / len(upstream_groups),
            4,
        ),
        "comparison_limit": "Owner/name coverage is exact for the selected headers. Overload candidates separately identify C++ const-only pairs, families covered by one Nim generic, and overloads whose parameter type is protected/private upstream; types, defaults, inheritance and build configurations still require compile tests.",
        "candidates": deficits,
        "upstream": entries,
    }
    rendered = json.dumps(result, indent=2, sort_keys=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(
        f"upstream signatures: {result['upstream_callable_signatures']}\n"
        f"owner/name groups: {result['upstream_owner_name_groups']}\n"
        f"raw declarations: {result['nim_raw_procedure_declarations']}\n"
        f"candidate groups: {result['overload_count_candidate_groups']}\n"
        f"candidate overload difference: {result['overload_count_candidate_difference']}"
    )


if __name__ == "__main__":
    main()
