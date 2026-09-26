"""Exercise an already-installed wheel from outside the source tree.

Run with the interpreter of a fresh wheel install, not an editable environment.
This is also used by the release CI job; it never installs or publishes packages.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    env = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    with tempfile.TemporaryDirectory(prefix="simdiff-release-") as directory:
        work = Path(directory)
        shutil.copytree(root / "fixtures", work / "fixtures")

        def run(args, expected=0):
            result = subprocess.run(
                [sys.executable, *args],
                cwd=work,
                env=env,
                text=True,
                capture_output=True,
                check=False,
                timeout=60,
            )
            if result.returncode != expected:
                raise RuntimeError(
                    f"{args}: expected exit {expected}, got {result.returncode}\n"
                    f"{result.stdout}\n{result.stderr}"
                )
            return result

        result = run(["-c", "import simdiff; print(simdiff.__file__)"])
        imported = Path(result.stdout.strip()).resolve()
        if imported.is_relative_to(root / "src") or "site-packages" not in imported.parts:
            raise RuntimeError(f"Expected installed wheel, imported source: {imported}")
        for name, expected in (
            ("good", 0),
            ("filters_removed", 1),
            ("inertia_changed", 1),
            ("collisions_disabled", 1),
        ):
            result = run(
                ["-m", "simdiff", "fixtures/good.xml", f"fixtures/{name}.xml", "--format", "json"],
                expected,
            )
            report = json.loads(result.stdout)
            assert report["status"] == ("no_findings" if expected == 0 else "differences")
            print(f"PASS: wheel {name}, exit {expected}")
        run(
            [
                "-m",
                "simdiff",
                "snapshot",
                "fixtures/mesh_enabled.xml",
                "--probe",
                "-o",
                "snapshot.json",
            ]
        )
        result = run(
            [
                "-m",
                "simdiff",
                "fixtures/mesh_enabled.xml",
                "snapshot.json",
                "--probe",
                "--format",
                "json",
            ]
        )
        assert json.loads(result.stdout)["status"] == "no_findings"
        result = run(["-m", "simdiff", "fixtures/good.xml", "missing.xml", "--format", "json"], 2)
        assert json.loads(result.stdout)["status"] == "error"
        result = run([str(root / "examples/runtime_collision.py")])
        assert "collision.body_mask_blocks_pair" in result.stdout
        assert "collision.probe_contact" in result.stdout
        print("PASS: installed-wheel snapshot, error contract and runtime collision example")


if __name__ == "__main__":
    main()
