# Contributing to simdiff

The most useful contribution is a real model case: what you converted, what you
expected to remain unchanged, and what simdiff reported. Useful findings, clean
reports, false positives and attempts blocked by unsupported inputs all help us
decide what to work on.

## Report a case

Use the [model-case form](https://github.com/ArithmancyAI/simdiff/issues/new?template=model-case.yml)
or [open a regular issue](https://github.com/ArithmancyAI/simdiff/issues/new).
Include the exact command, exit code, output, and versions of simdiff, Python,
MuJoCo and the converter/target engine. Explain which changes were intentional.
If possible, include a small, redistributable reproduction with its required assets.

Only share models and reports you have permission to publish. Reports can contain
model names, paths and physical properties; redact those as needed. A failed run
or a description without attachments is still useful. For a private conversation,
email [mustafa@arithmancy.ai](mailto:mustafa@arithmancy.ai).

To record your environment from the virtual environment used to run simdiff:

```bash
simdiff --version
python --version
python -m pip show mujoco
```

## Develop locally

From the repository root with Python 3.11–3.13 and `uv` installed:

```bash
uv sync --locked
uv run pytest -q
uv run ruff check .
uv run ruff format --check .
uv build
```

The [CI workflow](.github/workflows/simdiff.yml) covers Linux/macOS, Python
3.11–3.13 and MuJoCo 3.9/3.14, plus minimum dependencies and the built wheel.
It uploads package artifacts but does not publish to PyPI.

Keep fixes focused on the three selected checks. For a comparison bug, include a
small fixture that reproduces it and a regression test. Add unsupported inputs as
explicit errors rather than returning a clean report. Describe changes to the
JSON report or exit-code contract in your pull request.

The fixtures are original synthetic models; regenerate them with
`python fixtures/generate.py`. Do not add third-party robot assets without clear
redistribution permission. New engine adapters and broader checks should start
with an issue describing a real blocked workflow.
