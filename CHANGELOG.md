# Changelog

## 0.1.0b1 — experimental beta

- Compare selected compiled actuator-filter, body-inertia and collision properties.
- Accept MJCF, URDF and versioned snapshots of actual host MuJoCo models.
- Produce readable or JSON reports with exit codes for clean, changed and
  incomplete comparisons; support exact-value allowlists for intentional changes.
- Optionally compare contact presence at one matching pose without stepping physics.
- Include synthetic fixtures, a compiled-collision example and a CI workflow example.
- Handle float32 frame rounding and full-tensor inertia comparisons while rejecting
  unsupported frames, identities and actuator transmissions.

Supported range: Python 3.11–3.13, MuJoCo 3.9–3.14, Linux and macOS.
Full Newton/Warp adapters and GPU/device-model inspection are outside this beta.
See the [validation record](docs/validation.md) for evidence and limitations.
