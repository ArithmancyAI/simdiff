"""Three checks, with conservative identity and frame requirements."""

from itertools import combinations

import numpy as np

from .precision import (
    FRAME_POSITION_ATOL,
    FRAME_ROTATION_ATOL,
    FRAME_RTOL,
    poses_match,
    positions_match,
    rotations_match,
    tensors_match,
)
from .schema import Allowlist, Finding, InputError, Snapshot

RANK = {"none": 99, "info": 0, "warn": 1, "breaking": 2}
LIMITATIONS = [
    "Only actuator activation dynamics, body inertial properties and collision configuration are checked.",
    "Both file inputs use MuJoCo's compiler; other engines' importers are not executed.",
    "Snapshots describe the captured MjModel, not uninspected device arrays or custom callbacks.",
    "Collision pairs describe filter eligibility, not geometric reachability or all collider support.",
    "Mesh surfaces, solver settings, joint limits, damping, friction, sensors and trajectories are not compared.",
    "A clean report is not proof of whole-model equivalence, task success or physical accuracy.",
]


def eligible_pairs(snapshot):
    """Filter-eligible pairs, independent of pose and compiled broadphase caches."""
    if not snapshot.collision.enabled:
        return set()
    if len(snapshot.geoms) > 1500:
        raise InputError("v0 collision enumeration supports at most 1500 geoms per model")
    pairs = {tuple(sorted(pair)) for pair in snapshot.collision.explicit_pairs}
    excluded = {tuple(sorted(pair)) for pair in snapshot.collision.excludes}
    for a, b in combinations(sorted(snapshot.geoms), 2):
        ga, gb = snapshot.geoms[a], snapshot.geoms[b]
        if tuple(sorted([ga.body, gb.body])) in excluded:
            continue
        ba, bb = snapshot.bodies[ga.body], snapshot.bodies[gb.body]
        if ba.weld == bb.weld:
            continue
        if snapshot.collision.filter_parent and "world" not in (ba.weld, bb.weld):
            wa, wb = snapshot.bodies[ba.weld], snapshot.bodies[bb.weld]
            if (
                snapshot.bodies[wa.parent].weld == bb.weld
                or snapshot.bodies[wb.parent].weld == ba.weld
            ):
                continue
        if (ga.contype & gb.conaffinity) or (gb.contype & ga.conaffinity):
            pairs.add((a, b))
    return pairs


def compare(before, after, *, rtol=1e-6, atol=1e-12, allowlist=None, probe=False):
    """Return a JSON-ready report; incomplete/unsupported comparisons raise InputError."""
    before, after = Snapshot.model_validate(before), Snapshot.model_validate(after)
    if not np.isfinite([rtol, atol]).all() or min(rtol, atol) < 0:
        raise InputError("Tolerances must be finite and nonnegative")
    findings = []

    def add(code, entity, field, a, b, why, severity="breaking"):
        findings.append(
            Finding(
                code=code, entity=entity, field=field, before=a, after=b, why=why, severity=severity
            )
        )

    def numeric(a, b):
        a, b = np.asarray(a), np.asarray(b)
        return bool(np.all(np.abs(a - b) <= atol + rtol * np.maximum(np.abs(a), np.abs(b))))

    if before.mujoco_version != after.mujoco_version:
        add(
            "input.compiler_version",
            "model",
            "mujoco_version",
            before.mujoco_version,
            after.mujoco_version,
            "Compiler versions differ; defaults and preprocessing may differ.",
            "warn",
        )

    # A per-body tensor has meaning only in the same body frame. Do not mistake
    # an unsupported frame change or rigid-body fusion for an inertia comparison.
    for name in sorted(before.bodies.keys() & after.bodies.keys()):
        a, b = before.bodies[name], after.bodies[name]
        if (
            a.parent != b.parent
            or not positions_match(a.position, b.position)
            or not rotations_match(a.rotation, b.rotation)
        ):
            raise InputError(
                f"Body {name}: parent/body frame changed; v0 requires aligned body frames"
            )
    for kind, old, new in (
        ("body", before.bodies, after.bodies),
        ("actuator", before.actuators, after.actuators),
        ("geom", before.geoms, after.geoms),
    ):
        for name in sorted(old.keys() - new.keys()):
            visual = (
                kind == "geom"
                and not (old[name].contype or old[name].conaffinity)
                and not any(name in pair for pair in before.collision.explicit_pairs)
            )
            add(
                f"{kind}.removed",
                f"{kind}:{name}",
                "presence",
                True,
                False,
                "An entity is absent under its stable identity; review deletion or supply a name mapping.",
                "info" if visual else "breaking",
            )
        for name in sorted(new.keys() - old.keys()):
            add(
                f"{kind}.added",
                f"{kind}:{name}",
                "presence",
                False,
                True,
                "An entity was added under a new identity; review its intended role.",
                "warn",
            )

    if len(before.actuators) != len(after.actuators):
        add(
            "actuator.count",
            "model",
            "actuator_count",
            len(before.actuators),
            len(after.actuators),
            "The actuator inventory changed, which can change the control interface.",
        )
    for name in sorted(before.actuators.keys() & after.actuators.keys()):
        a, b = before.actuators[name], after.actuators[name]
        for field, why in (
            ("target", "The named actuator now routes to a different joint/transmission."),
            ("dyntype", "The command is processed by different activation dynamics."),
            ("actnum", "The number of activation states changed."),
            ("actearly", "The actuator uses activation values from a different point in the step."),
        ):
            if field == "actearly" and not (a.actnum or b.actnum):
                continue
            if getattr(a, field) != getattr(b, field):
                add(
                    f"actuator.{field}",
                    f"actuator:{name}",
                    field,
                    getattr(a, field),
                    getattr(b, field),
                    why,
                )
        # The first parameter is the time constant of the supported filters;
        # unused padding and NONE/INTEGRATOR dynprm do not alter semantics.
        if a.dyntype in (2, 3) and b.dyntype in (2, 3) and not numeric(a.dynprm[0], b.dynprm[0]):
            add(
                "actuator.filter_time_constant",
                f"actuator:{name}",
                "dynprm[0]",
                a.dynprm[0],
                b.dynprm[0],
                "The filter time constant changed (seconds).",
            )

    for name in sorted(before.bodies.keys() & after.bodies.keys() - {"world"}):
        a, b = before.bodies[name], after.bodies[name]
        for field, why in (
            ("mass", "Body mass changed (kg)."),
            ("com", "The center of mass moved in the body frame (m)."),
            ("inertia", "The full inertia tensor about the COM changed in the body frame (kg m²)."),
        ):
            equal = (
                tensors_match(a.inertia, b.inertia, rtol=rtol, atol=atol)
                if field == "inertia"
                else numeric(getattr(a, field), getattr(b, field))
            )
            if not equal:
                add(
                    f"inertia.{field}",
                    f"body:{name}",
                    field,
                    getattr(a, field),
                    getattr(b, field),
                    why,
                )

    for name in sorted(before.geoms.keys() & after.geoms.keys()):
        a, b = before.geoms[name], after.geoms[name]
        for field in ("contype", "conaffinity", "kind", "body", "group"):
            if getattr(a, field) != getattr(b, field):
                why = (
                    "Geom group is a visualization/inertia-inference group, not a collision mask."
                    if field == "group"
                    else "Collision configuration changed; pair findings describe filter eligibility."
                )
                add(
                    f"collision.{field}",
                    f"geom:{name}",
                    field,
                    getattr(a, field),
                    getattr(b, field),
                    why,
                    "info" if field == "group" else "warn",
                )
    for field in ("enabled", "filter_parent", "excludes", "explicit_pairs"):
        a, b = getattr(before.collision, field), getattr(after.collision, field)
        if field in ("excludes", "explicit_pairs"):
            a = [list(p) for p in sorted({tuple(sorted(p)) for p in a})]
            b = [list(p) for p in sorted({tuple(sorted(p)) for p in b})]
        if a != b:
            add(
                f"collision.{field}",
                "model",
                field,
                a,
                b,
                "Collision policy changed; inspect the affected filter-eligible pairs.",
                "breaking" if field == "enabled" else "warn",
            )

    old_pairs, new_pairs = eligible_pairs(before), eligible_pairs(after)
    for pair in sorted(old_pairs ^ new_pairs):
        add(
            "collision.pair_eligibility",
            "pair:" + repr(list(pair)),
            "eligible",
            pair in old_pairs,
            pair in new_pairs,
            "This geom pair's collision-filter eligibility changed; this does not predict task success.",
        )

    for side, snapshot, pairs in (("before", before, old_pairs), ("after", after, new_pairs)):
        active_geoms = {name for pair in pairs for name in pair}
        for name in sorted(active_geoms):
            if snapshot.geoms[name].mesh_data == "missing":
                add(
                    "collision.mesh_data_missing",
                    f"{side}/geom:{name}",
                    "mesh_data",
                    None,
                    "missing",
                    "An eligible mesh has no compiled hull graph/polygon data; "
                    "contact support depends on the collider. Recompile with intended masks and probe.",
                    "warn",
                )
        explicit = {tuple(sorted(pair)) for pair in snapshot.collision.explicit_pairs}
        for a, b in sorted(pairs - explicit):
            ba, bb = (
                snapshot.bodies[snapshot.geoms[a].body],
                snapshot.bodies[snapshot.geoms[b].body],
            )
            if not ((ba.contype & bb.conaffinity) or (bb.contype & ba.conaffinity)):
                add(
                    "collision.body_mask_blocks_pair",
                    f"{side}/pair:" + repr([a, b]),
                    "broadphase_eligible",
                    True,
                    False,
                    "Geom masks allow this pair, but cached body masks reject it before narrowphase; "
                    "capture after runtime edits or recompile with intended masks.",
                    "breaking" if side == "after" else "warn",
                )

    if probe:
        if before.probe is None or after.probe is None:
            raise InputError("Contact probing requires probes in both snapshots")
        if before.probe.poses.keys() != after.probe.poses.keys() or any(
            not poses_match(pose, after.probe.poses[name])
            for name, pose in before.probe.poses.items()
        ):
            raise InputError(
                "Contact probes use different body poses; supply matching configurations"
            )
        a = {tuple(sorted(p)) for p in before.probe.contacts}
        b = {tuple(sorted(p)) for p in after.probe.contacts}
        for pair in sorted(a ^ b):
            add(
                "collision.probe_contact",
                "pair:" + repr(list(pair)),
                "contact_at_pose",
                pair in a,
                pair in b,
                "Contact presence differs at this one pose; absence here does not prove a pair can never collide.",
                "warn",
            )

    if allowlist is not None:
        allowlist = Allowlist.model_validate(allowlist)
        used = set()
        for finding in findings:
            for i, rule in enumerate(allowlist.changes):
                if all(
                    getattr(finding, field) == getattr(rule, field)
                    for field in ("code", "entity", "field", "before", "after")
                ):
                    finding.accepted, finding.reason = True, rule.reason
                    used.add(i)
        if len(used) != len(allowlist.changes):
            raise InputError(
                "Allowlist contains unmatched changes; update it to the current comparison"
            )
    findings.sort(key=lambda f: (-RANK[f.severity], f.code, f.entity, f.field))
    counts = {
        severity: sum(f.severity == severity and not f.accepted for f in findings)
        for severity in ("info", "warn", "breaking")
    }
    accepted = sum(f.accepted for f in findings)
    return {
        "schema_version": 1,
        "status": "differences"
        if any(counts.values())
        else "accepted_changes"
        if accepted
        else "no_findings",
        "scope": ["actuator_filters", "inertia", "collision_configuration"],
        "before": {"label": before.label, "mujoco_version": before.mujoco_version},
        "after": {"label": after.label, "mujoco_version": after.mujoco_version},
        "tolerance": {"rtol": rtol, "atol": atol, "inertia_comparison": "frobenius_norm"},
        "frame_tolerance": {
            "position_rtol": FRAME_RTOL,
            "position_atol_m": FRAME_POSITION_ATOL,
            "rotation_atol": FRAME_ROTATION_ATOL,
        },
        "summary": {**counts, "accepted": accepted},
        "coverage": {
            "bodies_compared": len(before.bodies.keys() & after.bodies.keys()) - 1,
            "actuators_compared": len(before.actuators.keys() & after.actuators.keys()),
            "geoms_compared": len(before.geoms.keys() & after.geoms.keys()),
            "eligible_pairs_before": len(old_pairs),
            "eligible_pairs_after": len(new_pairs),
            "contact_probe": probe,
        },
        "findings": [f.model_dump() for f in findings],
        "limitations": LIMITATIONS,
    }


def exit_code(report, fail_on="breaking"):
    return int(
        any(not f["accepted"] and RANK[f["severity"]] >= RANK[fail_on] for f in report["findings"])
    )
