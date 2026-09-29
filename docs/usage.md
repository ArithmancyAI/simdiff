# Usage reference

Start with the [README quick start](../README.md#quick-start) for installation and
a complete example. This reference describes inputs, tolerances and CI behavior.

## What inputs mean

`simdiff BEFORE AFTER` accepts MJCF (`.xml`, `.mjcf`), URDF (`.urdf`) and versioned
simdiff snapshots (`.json`), in any combination. XML/URDF inputs are compiled with
**MuJoCo on both sides**. They do not automatically expose what another engine's
importer changed. Assets/includes must be available at their normal paths.

For a live compiled model, capture it **after** import and runtime modifications:

```python
from simdiff import capture, save

# source_model and converted_model are actual mujoco.MjModel instances supplied
# by your application/importer, after the changes you want to inspect.
save(capture(source_model, label="source"), "before.json")
save(capture(converted_model, label="converted"), "after.json")
```

```bash
simdiff before.json after.json --format json
simdiff snapshot model.xml -o compiled.json
```

Snapshot capture does not recompile the model. Exporting to XML and loading it
again can rebuild collision data and hide a runtime defect. Snapshots describe the
captured host `MjModel`; they cannot certify that a GPU/device model matches it.
Full Newton/Warp adapters and device-array inspection are deferred.

### Identities and unsupported inputs

Bodies, geoms and actuators need unique stable names. Actuators must reference named
joints. Names establish correspondence, never coincident array positions.
For an importer that renames/drops names, supply its verified mappings:

```python
snapshot = capture(
    converted_model,
    names={
        "body": {3: "payload"},
        "geom": {5: "payload_geom"},
        "actuator": {0: "drive_0"},
        "joint": {1: "joint_0"},
    },
)
```

These numbers are illustrative, not a mapping to use blindly. Mapping values are
source identities; keys are the target model's compiled IDs.

v0 supports named joint/joint-in-parent transmissions and `none`, `integrator`,
`filter`, `filterexact` activation dynamics. Other actuator dynamics/transmissions,
SDF plugins, flexible bodies, ambiguous names, incompatible body frames, malformed
snapshots and missing assets are **errors (exit 2)**, not clean reports. It supports
up to 1,500 geoms per model. URDF uses MuJoCo's import semantics; plain URDF does not
describe MuJoCo actuator filters, and fixed-body fusion may alter the body inventory.
This is not a general URDF-to-MJCF equivalence checker.

## The three checks

| Check | Compared | Interpretation |
| --- | --- | --- |
| Actuator filters | Named inventory, joint routing, activation type/count, filter time constant, early activation | Removed filters or changed active dynamics are breaking. Unused `dynprm` padding is ignored. |
| Inertia | Per-body mass, COM and full inertia tensor about the COM, in a common body frame | Numeric differences above tolerance are breaking. Principal-axis reorderings and quaternion sign changes alone pass. |
| Collision configuration | Masks, explicit pairs, exclusions, parent filtering, contact enablement, cached body masks, compiled mesh data | Changed pair eligibility and cached masks blocking intended pairs are breaking. Missing mesh data is a warning requiring inspection, not proof that every collider fails. |

MuJoCo `geom.group` is a visualization/inertia-inference group, not a collision mask;
a group-only change is informational. Pair eligibility accounts for same/welded
bodies, the parent filter, the world exception, exclusions and explicit pairs.
It does not prove geometric reachability or cover every collider/plugin override.
Geom shape/mesh equivalence, solver/integrator settings, joint limits, damping,
friction, cameras, sensors, hardware and trajectories are outside these checks.

Numerical comparison uses a symmetric tolerance:
`abs(a-b) <= atol + rtol * max(abs(a), abs(b))`, element by element.
Defaults are `rtol=1e-6`, `atol=1e-12` in SI units; customize with `--rtol`/`--atol`.
For inertia tensors, compare Frobenius norms instead:
`norm(A-B) <= atol + rtol * max(norm(A), norm(B))`. This is invariant to a common
rotation of the coordinate basis and avoids overstating errors in near-zero
off-diagonal components. The small absolute tolerance still detects changes in
tiny inertia tensors.

Frame alignment has its own fixed float32 representation allowance, recorded in
every report: position `rtol=2.384185791015625e-7`, `atol=1e-7 m` (0.1 micrometre),
and rotation-matrix entry `atol=2.384185791015625e-7`. These also apply to probe
poses. Quaternion normalization removes small unit-length roundoff; invalid
quaternions are errors. Physics `--rtol`/`--atol` options cannot loosen frame
alignment. Reparenting or changing a common body's frame beyond this allowance
is explicitly unsupported instead of producing a misleading tensor diff.

### Optional contact probe

`--probe` compares contact **presence by geom pair** at one matching pose. File
inputs use the compiled default configuration. For a runtime configuration use
`capture(model, probe=True, data=data)` on both sides. Data is copied; capture does
not step physics or change the caller's state. Captured body poses must agree.

Contact differences are warnings. One pose cannot establish that a pair can never
collide, and differing contact counts are not a measure of physics quality. This
probe is useful evidence alongside configuration findings, not trajectory replay.

## Reports and CI

Every finding has `code`, `severity`, `entity`, `field`, `before`, `after` and `why`.
Reports include a schema version, compiler versions, tolerances, coverage and scope
limitations. JSON output is stable and contains no timestamps. Results with
accepted changes retain the findings and reasons.

| Exit | Meaning |
| --- | --- |
| 0 | Completed; no unaccepted findings at or above `--fail-on`. |
| 1 | Completed; findings reached the configured severity threshold. |
| 2 | Invalid input, unsupported comparison, or operational error. |

`--fail-on breaking` is the default; alternatives are `warn`, `info` and `none`.
`none` preserves findings and never turns incomplete comparisons into success.
`--format json` writes a JSON error for load/comparison failures; argument syntax
errors follow argparse's stderr/exit-2 convention.

See [the GitHub Action example](../examples/github-action.yml). The workflow always
uploads the JSON report, including when the gate fails. It is an example for a
simdiff checkout; replace its fixture arguments with your own model paths.

### Explicitly accepted changes

Use `--allowlist accepted.json` to record an intentional change. Each entry must
match the exact code, identity, field and before/after values and provide a reason:

```json
{
  "schema_version": 1,
  "changes": [{
    "code": "inertia.mass",
    "entity": "body:payload",
    "field": "mass",
    "before": 0.01,
    "after": 0.02,
    "reason": "Approved payload replacement."
  }]
}
```

Copy values from a reviewed JSON report. Wildcards are not supported. Unmatched
entries are errors so stale approvals cannot silently cover a different change.
`breaking` means the selected property/contract changed, not that the robot is
unsafe or the task will fail.

## Troubleshooting

| Symptom | Next step |
| --- | --- |
| `python3.12: command not found` | Use an installed Python 3.11 or 3.13 instead; Python 3.14 is outside this beta's declared range. |
| `externally-managed-environment` during installation | Create and activate the virtual environment from the [quick start](../README.md#quick-start), then run `python -m pip install .` inside it. |
| `simdiff: command not found` | Activate `.venv` in this checkout. `python -m simdiff --version` checks the active interpreter's installation. |
| Missing XML, mesh or include file | Keep the model's asset/include layout intact and pass the correct model path. Missing assets make the comparison incomplete. |
| `Unnamed ...` or duplicate identity | Add stable names on both sides, or provide verified importer name mappings through `capture(names=...)`. Array order is not an identity mapping. |
| `v0 requires a named joint transmission` | Tendon and other non-joint transmissions are unsupported. This is an exit-2 result, not a passing check. |
| Incompatible body frames or probe poses | Confirm the models use matching body frames and the probes describe the same pose. Raising property tolerances does not bypass frame alignment. |
| Command exits 1 after printing findings | The comparison completed and reached the failure threshold. Review the findings; use an exact allowlist only for intentional changes. |

If you are still blocked, [share the failed run](https://github.com/ArithmancyAI/simdiff/issues/new?template=model-case.yml)
with the command, versions and error. A reduced reproduction helps, but is optional.

## Technical references

- [MuJoCo collision selection](https://mujoco.readthedocs.io/en/stable/computation/index.html#collision-detection)
- [Compiled model fields](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html)
- [Geom attributes](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom)
