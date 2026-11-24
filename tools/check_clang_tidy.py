#!/usr/bin/env python3
"""按 .clangd 配置扫描自研代码（User 与 Modules/WDR-Xrobot-org）的 clang-tidy 告警。"""

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import yaml


DIAG_RE = re.compile(r"^(.+?):(\d+):(\d+): (warning|error): (.+?) \[([^\]]+)\]$")


def load_clangd_config(clangd_path: Path):
    """解析 .clangd 主配置及 If.PathMatch 条件规则。"""
    docs = list(yaml.safe_load_all(clangd_path.read_text(encoding="utf-8")))
    if not docs or not isinstance(docs[0], dict):
        raise RuntimeError(f"Invalid .clangd config: {clangd_path}")

    root_ct = docs[0].get("Diagnostics", {}).get("ClangTidy", {})
    base_add = list(root_ct.get("Add", []))
    base_remove = list(root_ct.get("Remove", []))
    check_options = [
        {"key": str(k), "value": str(v)}
        for k, v in root_ct.get("CheckOptions", {}).items()
    ]

    skip_patterns = []
    conditional_rules = []
    for doc in docs[1:]:
        if not isinstance(doc, dict):
            continue
        cond = doc.get("If", {})
        raw_match = cond.get("PathMatch", [])
        if isinstance(raw_match, str):
            raw_match = [raw_match]
        patterns = [re.compile(p) for p in raw_match]
        ct = doc.get("Diagnostics", {}).get("ClangTidy", {})
        add_list = list(ct.get("Add", []))
        remove_list = list(ct.get("Remove", []))
        if "*" in remove_list:
            skip_patterns.extend(patterns)
        else:
            conditional_rules.append((patterns, add_list, remove_list))

    return base_add, base_remove, check_options, skip_patterns, conditional_rules


def discover_system_includes() -> list[str]:
    """从 arm-none-eabi-g++ 查询系统头文件路径。"""
    cmd = [
        "arm-none-eabi-g++",
        "-mcpu=cortex-m7",
        "-mfpu=fpv5-d16",
        "-mfloat-abi=hard",
        "-std=gnu++20",
        "-E",
        "-x",
        "c++",
        "-v",
        "/dev/null",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    includes = []
    in_search = False
    for line in proc.stderr.splitlines():
        if line.strip() == "#include <...> search starts here:":
            in_search = True
            continue
        if line.strip() == "End of search list.":
            break
        if in_search:
            path = os.path.normpath(line.strip())
            if path:
                includes.append(path)
    return includes


def matches_any(rel_posix: str, patterns: list[re.Pattern]) -> bool:
    return any(p.fullmatch(rel_posix) for p in patterns)


def resolve_module_prefixes(modules: list[str]) -> list[str]:
    prefixes = []
    for mod in modules:
        norm = mod.strip("/")
        if norm == "User" or norm.startswith("User/") or norm.startswith("Modules/"):
            prefixes.append(norm)
        else:
            prefixes.append(f"Modules/WDR-Xrobot-org/{norm}")
    return prefixes


def in_module_prefix(rel_posix: str, prefixes: list[str]) -> bool:
    return any(rel_posix == p or rel_posix.startswith(p + "/") for p in prefixes)


def build_tidy_config_json(
    rel_posix: str,
    base_add: list[str],
    base_remove: list[str],
    check_options: list[dict[str, str]],
    conditional_rules: list[tuple[list[re.Pattern], list[str], list[str]]],
) -> str:
    checks = ["-*"] + list(base_add) + [f"-{r}" for r in base_remove]
    for patterns, add_list, remove_list in conditional_rules:
        if matches_any(rel_posix, patterns):
            checks.extend(add_list)
            checks.extend(f"-{r}" for r in remove_list)
    return json.dumps(
        {
            "Checks": ",".join(checks),
            "CheckOptions": check_options,
        }
    )


def module_bucket(rel_posix: str) -> str:
    parts = rel_posix.split("/")
    if parts[0] == "User":
        return "User"
    if len(parts) >= 3 and parts[0] == "Modules":
        return "/".join(parts[:3])
    return parts[0]


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    default_jobs = min(8, os.cpu_count() or 4)

    parser = argparse.ArgumentParser(
        description="按 .clangd 规则检查 User 与 Modules/WDR-Xrobot-org 的 clang-tidy 告警"
    )
    parser.add_argument(
        "--build-dir",
        default="build/Debug",
        help="包含 compile_commands.json 的构建目录（默认: build/Debug）",
    )
    parser.add_argument(
        "--module",
        action="append",
        default=[],
        help="只检查指定模块（可重复，如 User、Gimbal 或 Modules/WDR-Xrobot-org/Gimbal）",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=default_jobs,
        help=f"并行 clang-tidy 任务数（默认: {default_jobs}）",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="仅输出统计摘要，不打印逐条告警",
    )
    args = parser.parse_args()

    build_dir = Path(args.build_dir)
    if not build_dir.is_absolute():
        build_dir = root / build_dir
    compile_db = build_dir / "compile_commands.json"
    if not compile_db.is_file():
        print(f"ERROR: compile_commands.json not found: {compile_db}", file=sys.stderr)
        return 2

    (
        base_add,
        base_remove,
        check_options,
        skip_patterns,
        conditional_rules,
    ) = load_clangd_config(root / ".clangd")
    sys_includes = discover_system_includes()

    entries = json.loads(compile_db.read_text(encoding="utf-8"))
    module_prefixes = resolve_module_prefixes(args.module)

    # 选择 User/ 与 Modules/WDR-Xrobot-org/ 下的编译单元。
    # 注意：User/app_main.cpp 会通过 User/xrobot_main.hpp 引入全部纯头文件模块，
    # 因此即使它在 skip_patterns 中，仍需作为 TU 参与分析（其自身位置的告警会在后处理中过滤）。
    selected_tus: list[tuple[str, str]] = []
    seen_tus: set[str] = set()
    need_app_main = any(p.startswith("Modules/") for p in module_prefixes)
    for entry in entries:
        src_path = Path(entry["file"]).resolve()
        try:
            rel_posix = src_path.relative_to(root).as_posix()
        except ValueError:
            continue
        if not (
            rel_posix.startswith("User/")
            or rel_posix.startswith("Modules/WDR-Xrobot-org/")
        ):
            continue
        if "/vendor/" in rel_posix or rel_posix.startswith("vendor/"):
            continue
        if matches_any(rel_posix, skip_patterns) and rel_posix != "User/app_main.cpp":
            continue
        if module_prefixes:
            matched = in_module_prefix(rel_posix, module_prefixes)
            if not (matched or (need_app_main and rel_posix == "User/app_main.cpp")):
                continue
        if rel_posix not in seen_tus:
            seen_tus.add(rel_posix)
            selected_tus.append((str(src_path), rel_posix))

    if not selected_tus:
        print("No matching translation units found.")
        return 0

    header_filter = f"^{re.escape(str(root))}/(User|Modules/WDR-Xrobot-org)/"
    extra_args = ["--extra-arg=--target=arm-none-eabi"] + [
        f"--extra-arg=-isystem{inc}" for inc in sys_includes
    ]

    def run_tu(item: tuple[str, str]):
        src_str, rel_posix = item
        cfg_json = build_tidy_config_json(
            rel_posix, base_add, base_remove, check_options, conditional_rules
        )
        cmd = [
            "clang-tidy",
            "-p",
            str(build_dir),
            f"--config={cfg_json}",
            f"--header-filter={header_filter}",
            *extra_args,
            src_str,
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        diags = []
        for line in proc.stdout.splitlines():
            m = DIAG_RE.match(line)
            if not m:
                continue
            path_str, line_no, col_no, sev, msg, check = m.groups()
            try:
                diag_rel = Path(path_str).resolve().relative_to(root).as_posix()
            except ValueError:
                continue
            if not (
                diag_rel.startswith("User/")
                or diag_rel.startswith("Modules/WDR-Xrobot-org/")
            ):
                continue
            if matches_any(diag_rel, skip_patterns):
                continue
            if any(
                matches_any(diag_rel, pats) and check in rem
                for pats, _, rem in conditional_rules
            ):
                continue
            if module_prefixes and not in_module_prefix(diag_rel, module_prefixes):
                continue
            diags.append((diag_rel, int(line_no), int(col_no), sev, msg, check))
        return rel_posix, proc.returncode, proc.stderr, diags

    deduped: dict[tuple[str, int, int, str], tuple[str, int, int, str, str, str]] = {}
    tu_failures: list[tuple[str, int, str]] = []
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        for rel_posix, rc, stderr, diags in pool.map(run_tu, selected_tus):
            if rc != 0:
                tu_failures.append((rel_posix, rc, stderr))
            for diag in diags:
                deduped[(diag[0], diag[1], diag[2], diag[5])] = diag

    sorted_diags = sorted(deduped.values())
    if not args.summary_only:
        for diag_rel, line_no, col_no, sev, msg, check in sorted_diags:
            print(f"{diag_rel}:{line_no}:{col_no}: {sev}: {msg} [{check}]")

    by_check = Counter(d[5] for d in sorted_diags)
    by_module = Counter(module_bucket(d[0]) for d in sorted_diags)

    print(
        f"\nScanned {len(selected_tus)} TUs; "
        f"total diagnostics: {len(sorted_diags)}"
    )
    if by_check:
        print("\nBy check:")
        for check, count in by_check.most_common():
            print(f"  {count:5d}  {check}")
    if by_module:
        print("\nBy module:")
        for mod, count in by_module.most_common():
            print(f"  {count:5d}  {mod}")

    if tu_failures:
        print("\nclang-tidy failures:", file=sys.stderr)
        for rel_posix, rc, stderr in tu_failures:
            print(f"  {rel_posix} exited with {rc}", file=sys.stderr)
            if stderr.strip():
                print(stderr.strip(), file=sys.stderr)
        return 1

    return 0 if not sorted_diags else 1


if __name__ == "__main__":
    sys.exit(main())
