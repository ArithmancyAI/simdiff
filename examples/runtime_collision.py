"""A real CPU compile/runtime discrepancy using only original synthetic geometry."""

import tempfile
from pathlib import Path

import mujoco
import numpy as np

from simdiff import capture, compare, load, save
from simdiff.cli import table

fixtures = Path(__file__).resolve().parents[1] / "fixtures"
before = mujoco.MjModel.from_xml_path(str(fixtures / "mesh_enabled.xml"))
after = mujoco.MjModel.from_xml_path(str(fixtures / "mesh_disabled.xml"))

# Runtime edit does not rebuild body masks or mesh preprocessing.
after.geom_contype[:] = before.geom_contype
after.geom_conaffinity[:] = before.geom_conaffinity
assert np.array_equal(before.geom_contype, after.geom_contype)
assert np.array_equal(before.geom_conaffinity, after.geom_conaffinity)

out = Path(tempfile.mkdtemp(prefix="simdiff-demo-"))
save(capture(before, label="compiled with collisions", probe=True), out / "before.json")
save(capture(after, label="enabled after compilation", probe=True), out / "after.json")
report = compare(load(out / "before.json"), load(out / "after.json"), probe=True)
assert "collision.body_mask_blocks_pair" in {f["code"] for f in report["findings"]}
assert "collision.probe_contact" in {f["code"] for f in report["findings"]}
print(table(report))
print(f"\nSnapshots: {out}")
print(f"simdiff '{out / 'before.json'}' '{out / 'after.json'}' --probe --format json")
