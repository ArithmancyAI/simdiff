# Matching masks do not guarantee matching collision behavior

Worked example and local blog draft. The measurements below describe these
synthetic fixtures, not an engine ranking or a hardware experiment.

Robot-model conversion can change small properties with substantial behavioral
consequences. simdiff starts with three concrete checks: actuator activation
filters, inertial properties and collision configuration.

`fixtures/good.xml` has nine filtered actuators and a small payload. Its mutations
remove those filters, alter the payload's inertia tensor, or disable payload
collisions. MuJoCo compiles every file successfully. simdiff reports the selected
changes and returns a nonzero exit code suitable for CI.

The collision example goes a step further. `mesh_enabled.xml` and
`mesh_disabled.xml` contain the same original tessellated sphere, overlapping a
plane at their default pose. The only authored difference is whether the sphere
can collide when MuJoCo compiles the model.

Run:

```bash
python examples/runtime_collision.py
```

The script compiles both files, then copies the enabled model's geom masks into
the disabled model **after compilation**. It asserts both mask arrays match and
captures the actual compiled models to JSON. Cached body masks still differ.
The optional probe checks collision detection at the same body poses without
integrating physics: the enabled model reports the floor/shell pair; the edited
model does not. The report identifies the body-mask inconsistency, missing mesh
preprocessing where observed, and the contact difference at that pose.

The body-mask finding explains the broadphase rejection in this fixture. Missing
mesh preprocessing is reported conservatively: some colliders can still produce
contacts without a hull graph, so that property alone is not a universal proof
that a mesh cannot collide. Nor does one empty contact set establish that a pair
can never collide in other poses.

This is why snapshots are part of v0. Exporting a runtime model to XML and
recompiling it may rebuild the missing structures, making a file-only comparison
look healthy. Capturing the actual model preserves evidence that a text diff or
recompile can lose.

Inertia has a related representation problem: a change to principal-axis ordering
is not necessarily a change to the physical tensor. simdiff rotates the diagonal
principal inertia into the body frame before comparing it, and tests an equivalent
axis permutation as a passing case.

The tool reports selected differences, not their effect on a task. A dropped
filter does not by itself prove why a grasp failed. Two clean snapshots do not
establish agreement between a host model and a GPU runtime that was never inspected.

Run it on your own port, then share a useful finding or a failed attempt through
the hosting repository's issues or [mustafa@arithmancy.ai](mailto:mustafa@arithmancy.ai).
