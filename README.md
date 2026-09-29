# simdiff

[![CI](https://github.com/ArithmancyAI/simdiff/actions/workflows/simdiff.yml/badge.svg)](https://github.com/ArithmancyAI/simdiff/actions/workflows/simdiff.yml)

Catch changes in **actuator filters, body inertia and collision configuration**
after converting a robot model.

A model can load successfully after a conversion while its filters have been
removed, its inertia changed, or its collision rules altered. simdiff compares
compiled model properties, explains the selected differences, and returns an exit
code you can use in CI.

**Experimental beta: 0.1.0b1.** Python 3.11–3.13, MuJoCo 3.9–3.14, Linux and macOS.
Runs on CPU; no GPU or hardware required. Full Newton/Warp import adapters are not
included: file inputs are compiled with MuJoCo on both sides, while snapshots
capture an existing host MuJoCo model. See [input selection](#run-it-on-your-own-models).

## Quick start

Use an installed Python 3.11, 3.12 or 3.13; these commands use 3.12. From a terminal:

```bash
git clone https://github.com/ArithmancyAI/simdiff.git
cd simdiff
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install .
simdiff fixtures/good.xml fixtures/good.xml
```

The first comparison exits **0** and prints `No findings in the three selected checks.`
Then compare a fixture whose payload inertia was changed:

```bash
simdiff fixtures/good.xml fixtures/inertia_changed.xml
```

Expected output:

```text
SEVERITY  ENTITY  CHANGE  BEFORE -> AFTER
breaking  "body:payload"  inertia.inertia  [[1e-07, 0.0, 0.0], [0.0, 1e-07, 0.0], [0.0, 0.0, 1e-07]] -> [[4.46e-08, 0.0, 0.0], [0.0, 4.46e-08, 0.0], [0.0, 0.0, 5.73e-08]]
          The full inertia tensor about the COM changed in the body frame (kg m²).

1 breaking | 0 warn | 0 info | 0 accepted
Scope: actuator filters, inertia, collision configuration. A clean report is not whole-model equivalence.
```

This command intentionally exits **1**: it found a change, so installation worked.
`body:payload` identifies the affected body; `inertia.inertia` identifies the
property. The matrices are the full inertia tensors before and after, in kg m².
**Breaking** means a checked property changed beyond tolerance. It does not by
itself establish that a task fails or that a robot is unsafe.

Install from this checkout for now; we have not published a PyPI release. If a
command fails to run, see [troubleshooting](docs/usage.md#troubleshooting).

## Run it on your own models

Replace the two paths with your source and converted models:

```bash
simdiff path/to/before.xml path/to/after.xml
```

| What you have | What to provide |
| --- | --- |
| MJCF or URDF files before and after a conversion | Pass the two files. Both are compiled by MuJoCo; referenced meshes and includes must be available. |
| Actual `mujoco.MjModel` instances after import/runtime edits | Capture each model with the Python API, then compare the two JSON snapshots. [Snapshot example](docs/usage.md#what-inputs-mean). |
| Only another engine's native model or device arrays | v0 has no automatic adapter for these. File comparison cannot inspect changes that only exist in that runtime. |

Bodies, geoms and actuators need unique, stable names; actuators must reference
named joints. Tendon actuators and incompatible body frames are unsupported.
[Input requirements and limitations](docs/usage.md#identities-and-unsupported-inputs)
explain these cases and verified name mappings.

For a machine-readable report:

```bash
simdiff path/to/before.xml path/to/after.xml --format json > simdiff-report.json
```

| Exit | Meaning | What to do |
| --- | --- | --- |
| **0** | No unaccepted findings reached the selected failure threshold. | Inspect warnings and coverage; this is not whole-model equivalence. |
| **1** | Findings reached the threshold, which defaults to `breaking`. | Review the before/after values. Record intentional changes with an [allowlist](docs/usage.md#explicitly-accepted-changes). |
| **2** | Invalid input, unsupported comparison or an operational error. | Read the error and resolve it before treating the comparison as complete. |

## What gets checked

| Check | Example finding | Try the included mutation |
| --- | --- | --- |
| Actuator filters | Nine actuator filters were removed. | `simdiff fixtures/good.xml fixtures/filters_removed.xml` |
| Inertia | A body's mass, center of mass or inertia tensor changed. | `simdiff fixtures/good.xml fixtures/inertia_changed.xml` |
| Collision configuration | A previously eligible geom pair lost collision eligibility. | `simdiff fixtures/good.xml fixtures/collisions_disabled.xml` |

Each mutation exits 1. The optional `--probe` also compares contact presence at
one matching pose, without integrating physics. The
[worked example](docs/why-compiled-state-matters.md) shows why matching collision
masks can still hide a change in compiled collision data.

The checks do not cover solver/integrator settings, joint limits, damping,
friction, sensors, cameras, complete geometry equivalence, or trajectories.
simdiff does not judge physical accuracy or certify agreement between engines.
See the [full comparison rules](docs/usage.md#the-three-checks),
[CI example](examples/github-action.yml), and [validation record](docs/validation.md).

## Try a real case and tell us what happened

**Run it on your own port, then [share a model case](https://github.com/ArithmancyAI/simdiff/issues/new?template=model-case.yml).**
A useful finding, a clean report, a false positive, or an unsupported input are all
helpful. Include the command, versions, expected change and actual result. A small
reproduction or redacted report is enough; you do not need to share private models.
For a private conversation, email [mustafa@arithmancy.ai](mailto:mustafa@arithmancy.ai).

There is no telemetry or network access in the CLI. We learn from cases and
conversations you choose to share. [Contributing](CONTRIBUTING.md) covers development
and reporting bugs.

Apache-2.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE). The repository contains
original synthetic fixtures and excludes Project-X's robot assets, datasets,
proprietary replay and hardware work.
