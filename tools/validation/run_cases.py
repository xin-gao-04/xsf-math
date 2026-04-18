#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def load_cases(repo_root: Path) -> list[dict]:
    config_path = repo_root / "tools" / "validation" / "cases.json"
    with config_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)["cases"]


def run_command(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        text=True,
        capture_output=True,
        check=False,
    )


def parse_ctest_output(output: str) -> list[dict]:
    parsed = []
    pattern = re.compile(r"Test\s+#(\d+):\s+(.+?)\s+\.+\s+(Passed|Failed|Subprocess aborted)")
    for raw_line in output.splitlines():
        line = raw_line.strip()
        match = pattern.search(line)
        if match is None:
            continue
        parsed.append({
            "index": int(match.group(1)),
            "name": match.group(2).strip(),
            "passed": match.group(3) == "Passed",
            "line": line,
        })
    return parsed


def main() -> int:
    parser = argparse.ArgumentParser(description="Run xsf validation cases and collect artifacts.")
    parser.add_argument("--build-dir", default="build", help="CMake build directory.")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Validation output directory. Defaults to <build-dir>/validation.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip the cmake --build step.",
    )
    parser.add_argument(
        "--skip-ctest",
        action="store_true",
        help="Skip the ctest regression run.",
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="Generate HTML/Markdown reports after the run completes.",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    build_dir = (repo_root / args.build_dir).resolve()
    output_dir = (repo_root / args.output_dir).resolve() if args.output_dir else (build_dir / "validation").resolve()
    cases_root = output_dir / "cases"

    if output_dir.exists():
        shutil.rmtree(output_dir)
    cases_root.mkdir(parents=True, exist_ok=True)

    cases = load_cases(repo_root)
    index = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "build_dir": str(build_dir),
        "output_dir": str(output_dir),
        "cases": [],
        "ctest": {},
    }

    if not args.skip_build:
        build_result = run_command(["cmake", "--build", str(build_dir), "-j4"], repo_root)
        (output_dir / "build.log").write_text(build_result.stdout + build_result.stderr, encoding="utf-8")
        if build_result.returncode != 0:
            index["build"] = {
                "passed": False,
                "returncode": build_result.returncode,
                "log": "build.log",
            }
            (output_dir / "run_index.json").write_text(json.dumps(index, indent=2, ensure_ascii=False), encoding="utf-8")
            return build_result.returncode
        index["build"] = {
            "passed": True,
            "returncode": 0,
            "log": "build.log",
        }

    for case in cases:
        binary_path = build_dir / case["binary"]
        case_dir = cases_root / case["id"]
        case_dir.mkdir(parents=True, exist_ok=True)

        result = run_command(
            [
                str(binary_path),
                "--strict",
                "--output-dir",
                str(cases_root),
            ],
            repo_root,
        )
        console_log = result.stdout + result.stderr
        (case_dir / "console.log").write_text(console_log, encoding="utf-8")

        summary_path = case_dir / "run_summary.json"
        summary = {}
        if summary_path.exists():
            with summary_path.open("r", encoding="utf-8") as handle:
                summary = json.load(handle)

        index["cases"].append({
            "id": case["id"],
            "title": case["title"],
            "category": case["category"],
            "summary": case["summary"],
            "binary": case["binary"],
            "returncode": result.returncode,
            "passed": result.returncode == 0 and bool(summary.get("passed", True)),
            "artifact_dir": str(case_dir.relative_to(output_dir)),
            "console_log": str((case_dir / "console.log").relative_to(output_dir)),
            "run_summary": str(summary_path.relative_to(output_dir)) if summary_path.exists() else None,
        })

    if not args.skip_ctest:
        ctest_result = run_command(
            ["ctest", "--test-dir", str(build_dir), "--output-on-failure"],
            repo_root,
        )
        (output_dir / "ctest.log").write_text(ctest_result.stdout + ctest_result.stderr, encoding="utf-8")
        index["ctest"] = {
            "passed": ctest_result.returncode == 0,
            "returncode": ctest_result.returncode,
            "tests": parse_ctest_output(ctest_result.stdout + ctest_result.stderr),
            "log": "ctest.log",
        }

    (output_dir / "run_index.json").write_text(json.dumps(index, indent=2, ensure_ascii=False), encoding="utf-8")

    if args.report:
        report_script = repo_root / "tools" / "validation" / "build_report.py"
        report_result = run_command(
            [
                sys.executable,
                str(report_script),
                "--input-dir",
                str(output_dir),
            ],
            repo_root,
        )
        (output_dir / "report.log").write_text(report_result.stdout + report_result.stderr, encoding="utf-8")
        if report_result.returncode != 0:
            return report_result.returncode

    case_failures = [case for case in index["cases"] if not case["passed"]]
    ctest_failed = bool(index["ctest"]) and not index["ctest"].get("passed", False)
    return 1 if case_failures or ctest_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
