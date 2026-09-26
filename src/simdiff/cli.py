"""Portable CLI: JSON on stdout, diagnostics on stderr, meaningful exit codes."""

import argparse
import json
import sys

import mujoco

from . import __version__
from .compare import compare, exit_code
from .snapshot import load, read_json, save


def table(report):
    lines = ["SEVERITY  ENTITY  CHANGE  BEFORE -> AFTER"]
    for f in report["findings"]:
        severity = "accepted" if f["accepted"] else f["severity"]
        a, b = json.dumps(f["before"], ensure_ascii=True), json.dumps(f["after"], ensure_ascii=True)
        if f["field"] == "dyntype":
            types = {0: "none", 1: "integrator", 2: "filter", 3: "filterexact"}
            a, b = types[f["before"]], types[f["after"]]
        # JSON-encode external names to keep untrusted control characters out of terminals.
        entity = json.dumps(f["entity"], ensure_ascii=True)
        lines.append(f"{severity:9} {entity}  {f['code']}  {a} -> {b}")
        lines.append(f"          {f['why']}")
        if f["accepted"]:
            lines.append("          Accepted: " + json.dumps(f["reason"], ensure_ascii=True))
    if not report["findings"]:
        lines.append("No findings in the three selected checks.")
    counts = report["summary"]
    lines.extend(
        [
            "",
            " | ".join(f"{counts[k]} {k}" for k in ("breaking", "warn", "info", "accepted")),
            (
                "Scope: actuator filters, inertia, collision configuration. "
                "A clean report is not whole-model equivalence."
            ),
        ]
    )
    return "\n".join(lines)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    snapshot_mode = bool(argv and argv[0] == "snapshot")
    parser = argparse.ArgumentParser(
        prog="simdiff snapshot" if snapshot_mode else "simdiff",
        description="Compare actuator filters, inertia and collision configuration using MuJoCo.",
    )
    parser.add_argument("--version", action="version", version=f"simdiff {__version__}")
    parser.add_argument(
        "--probe", action="store_true", help="Compare contacts at the default/captured pose"
    )
    if snapshot_mode:
        parser.add_argument("model")
        parser.add_argument("-o", "--output", required=True)
    else:
        parser.add_argument("before")
        parser.add_argument("after")
        parser.add_argument("--format", choices=("table", "json"), default="table")
        parser.add_argument(
            "--fail-on", choices=("none", "info", "warn", "breaking"), default="breaking"
        )
        parser.add_argument("--allowlist", help="JSON list of exact accepted changes with reasons")
        parser.add_argument("--rtol", type=float, default=1e-6)
        parser.add_argument("--atol", type=float, default=1e-12)
    args = parser.parse_args(argv[1:] if snapshot_mode else argv)
    try:
        if snapshot_mode:
            save(load(args.model, probe=args.probe), args.output)
            print(f"Saved snapshot to {args.output}", file=sys.stderr)
            return 0
        report = compare(
            load(args.before, probe=args.probe),
            load(args.after, probe=args.probe),
            probe=args.probe,
            rtol=args.rtol,
            atol=args.atol,
            allowlist=read_json(args.allowlist) if args.allowlist else None,
        )
        print(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
            if args.format == "json"
            else table(report)
        )
        return exit_code(report, args.fail_on)
    except (ValueError, OSError, mujoco.FatalError) as exc:
        if not snapshot_mode and args.format == "json":
            print(json.dumps({"schema_version": 1, "status": "error", "error": str(exc)}))
        else:
            print(f"simdiff: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
