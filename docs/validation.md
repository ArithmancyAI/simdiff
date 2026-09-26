# Beta validation record — 26 September 2026

This record distinguishes local evidence from the hosted release gate. The public
package remains a MuJoCo-file/snapshot comparator. The private validation below
does not add Newton or GPU runtime support to the CLI.

## Precision fixes

The initial implementation rejected unchanged models after a float32 position
round trip because its frame-alignment guard used a hard 1 nm tolerance. Actual
importers also compose and subtract transforms, producing small cancellation
errors. Frame alignment now allows 0.1 micrometre absolute position error plus
two float32 epsilons of relative coordinate error. Rotation entries have a fixed
two-epsilon allowance. These bounds are independent of configurable property
tolerances, visible in JSON, and still reject meaningful frame changes.

The real arm conversion also exposed an inertia false positive: componentwise
relative errors overstated harmless rounding in a small off-diagonal entry.
Comparing full-tensor Frobenius norms removed that false positive while retaining
the real small-payload inertia change. The regression suite covers both outcomes.

## Real model imports

Local fresh Newton 1.5.1 imports used MuJoCo 3.11.0, MuJoCo-Warp 3.11.0 and
Warp 1.16.0 on CPU. Comparisons use the **host MjModel after conversion** and
explicit identity mappings from the importer. There are no new physics rollouts,
hardware comparisons or GPU/device-array claims.

| Case | Result in selected checks | Preparation / limits |
| --- | --- | --- |
| Internal nine-actuator arm | Nine removed filters, one payload tensor change; 19 breaking field findings. The spurious base-inertia finding is gone. | Private model; verified importer identity maps. Four omitted collision-disabled geoms are informational. |
| SO101 | Two breaking findings: camera-mount COM and inertia changed. | Added names to 30 anonymous elements before conversion; selected compiled source arrays verified unchanged. |
| UR5e | No breaking findings; a collision-exclusion policy warning and 20 informational omitted visual geoms. | Added names to 29 anonymous elements before conversion; selected compiled source arrays verified unchanged. |
| XArm7 | Explicitly unsupported: tendon-driven gripper actuator. | Added 15 names, but the joint-transmission restriction still prevents comparison. No clean report was issued. |

These are observations of selected model changes, not diagnoses of why an engine
behaves differently. Naming was necessary: the original anonymous SO101/UR5e
inputs did not support reliable automatic correspondence. v0 still requires
stable names or explicit mappings; it does not promise a frictionless run on
every robot. The source assets and the private conversion harness are not included
in the package. The synthetic regression suite is self-contained.

## Release gate

Local verification completed with **64 tests passing in each of six environments**:

| Platform | Python | MuJoCo | Dependency selection |
| --- | --- | --- | --- |
| macOS ARM64 | 3.11 | 3.9.0 | NumPy 1.26.0 / Pydantic 2.6.0 minimums |
| macOS ARM64 | 3.12 | 3.9.0 | Existing local environment |
| macOS ARM64 | 3.12 | 3.14.0 | Locked development environment |
| macOS ARM64 | 3.13 | 3.14.0 | Fresh wheel environment |
| Linux ARM64, Docker | 3.11 | 3.9.0 | NumPy 1.26.0 / Pydantic 2.6.0 minimums |
| Linux ARM64, Docker | 3.11 | 3.14.0 | Fresh wheel environment |

The built `0.1.0b1` wheel also passed the standalone release checks on macOS
and in both Linux containers: clean and mutated fixture exit codes, snapshot
round trip, contact probe, JSON input-error handling, and the runtime collision
example. Ruff lint/format checks, distribution builds and `actionlint` passed.

The workflow at [`.github/workflows/simdiff.yml`](../.github/workflows/simdiff.yml) runs:

- Linux/macOS × Python 3.11/3.12/3.13 × MuJoCo 3.9.0/3.14.0.
- A Python 3.11 minimum-dependency test with NumPy 1.26.0 and Pydantic 2.6.0.
- Locked lint/format checks, source distribution and wheel builds, and an
  installed-wheel CLI/snapshot/runtime-example check outside the source tree.

Local macOS checks and an actual Linux ARM64 Docker run provide evidence before
push. They do not establish that the GitHub-hosted Linux x86_64/macOS matrix has
passed: the first hosted run is still required before public publication. Windows,
Python 3.14+, MuJoCo 3.15+, GPU runtime verification and tendon actuators are outside
this beta's declared support. Independent user trials are still needed to measure
usefulness and false-positive rates in external workflows.
