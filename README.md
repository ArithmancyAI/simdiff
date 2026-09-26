# simdiff

Find changes in actuator filters, body inertia and collision configuration before
and after a robot-model conversion. Python CLI, CPU MuJoCo, readable output or JSON.

**0.1.0b1 is a scoped beta candidate and a demand probe.** It checks three specific classes of changes. It does not
judge physical accuracy, predict task success, compare trajectories or certify
that two engines are equivalent.

Supported release range: Python 3.11–3.13, MuJoCo 3.9–3.14, Linux and macOS.
See the [validation record](docs/validation.md) for tested combinations and known
onboarding limits. The hosted CI matrix must pass before a public release.

## Try it in 30 seconds

Clone the repository and install from its root:

```bash
git clone https://github.com/ArithmancyAI/simdiff.git
cd simdiff
python -m pip install .
simdiff fixtures/good.xml fixtures/inertia_changed.xml
```

The report identifies `body:payload`, prints the before/after inertia tensors and
explains the change. It exits **1**, intentionally: the fixture contains a breaking
change. This is an installable local package; a public PyPI release is not yet published.

```bash
# No changes: exit 0.
simdiff fixtures/good.xml fixtures/good.xml

# Nine activation filters removed: exit 1.
simdiff fixtures/good.xml fixtures/filters_removed.xml

# Payload/floor collision eligibility lost: exit 1.
simdiff fixtures/good.xml fixtures/collisions_disabled.xml

# Machine-readable report; still exits 1 for breaking findings.
simdiff fixtures/good.xml fixtures/filters_removed.xml --format json --fail-on breaking

# Matching mask values, different compiled state and contacts.
python examples/runtime_collision.py
```

**Run it on your own port, then talk to us:** [open a model-case issue](https://github.com/ArithmancyAI/simdiff/issues),
or email [mustafa@arithmancy.ai](mailto:mustafa@arithmancy.ai).
Tell us what you converted, whether you could run the command, and whether a finding
was useful. Private models are not required; a reduced case or report is enough.
There is no telemetry or network access in the CLI.

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

See [the GitHub Action example](examples/github-action.yml). The workflow always
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

## Development and release

```bash
uv sync --locked
uv run pytest -q
uv run ruff check .
uv build
```

Fixtures are original synthetic models; regenerate with `python fixtures/generate.py`.
The suite includes all three mutations, the runtime collision case, unchanged and
reordered inputs, equivalent inertia representations, explicit pairs/exclusions,
allowlists, malformed inputs, URDF loading and subprocess CLI exit-code checks.

The suite includes precision regressions for float32 positions/orientations,
transform cancellation, and inertia eigenbasis rounding, while retaining true
frame-change and small-inertia-change detection. The
[CI workflow](.github/workflows/simdiff.yml) runs a Linux/macOS, Python
3.11/3.12/3.13, MuJoCo 3.9/3.14 matrix, a minimum-dependency job, and installed-wheel
smoke tests. The workflow does not publish packages.

Read the [worked example / blog draft](docs/why-compiled-state-matters.md) and
[demand-probe plan](docs/demand-probe.md). PyPI publication and a public blog URL
remain release tasks. The package is self-contained and does not import Project-X.

Apache-2.0 applies to the new code, documentation and synthetic fixtures in this
repository; see [LICENSE](LICENSE). It does not change Project-X's license or
redistribute its robot assets, datasets, replay or hardware work.

Technical references: [MuJoCo collision selection](https://mujoco.readthedocs.io/en/stable/computation/index.html#collision-detection),
[compiled model fields](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html),
[geom attributes](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom).
