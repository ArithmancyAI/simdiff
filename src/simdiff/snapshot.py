"""Capture compiled state directly, including properties lost by XML export."""

import json
from pathlib import Path

import mujoco
import numpy as np

from .schema import InputError, Snapshot


def rotation(quat):
    quat = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or abs(norm - 1) > 1e-6:
        raise InputError("Model quaternion must have unit length (within float32 rounding)")
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, quat / norm)
    return result.reshape(3, 3)


def read_json(path):
    def unique_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise InputError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    def bad_constant(value):
        raise InputError(f"Non-finite JSON number: {value}")

    return json.loads(
        Path(path).read_text(), object_pairs_hook=unique_keys, parse_constant=bad_constant
    )


def capture(model, *, label="runtime", names=None, probe=False, data=None):
    """Capture a live MjModel after import/runtime edits, without recompiling it.

    ``names`` optionally maps kind -> compiled index -> stable source name.
    Callers must establish those identities using their importer's mappings.
    No positional correspondence is inferred between different models.
    A probe uses a copy of ``data`` or the compiled default configuration.
    """
    if model.nflex or model.nplugin:
        raise InputError("Flexible bodies and model plugins are outside v0's selected checks")
    names = names or {}
    if set(names) - {"body", "geom", "actuator", "joint"}:
        raise InputError("Name overrides support body, geom, actuator and joint only")

    def identities(kind, count, *, required=True):
        overrides = names.get(kind, {})
        if any(type(i) is not int or i < 0 or i >= count for i in overrides):
            raise InputError(f"Invalid {kind} index in name overrides")
        result = {}
        for i in range(count):
            name = overrides.get(i) or mujoco.mj_id2name(
                model, getattr(mujoco.mjtObj, f"mjOBJ_{kind.upper()}"), i
            )
            if kind == "body" and i == 0:
                if i in overrides and overrides[i] != "world":
                    raise InputError("The world body's identity must be world")
                name = "world"
            if not name:
                if not required:
                    continue
                raise InputError(f"Unnamed {kind} {i}; add a name or use capture(names=...)")
            if not isinstance(name, str) or name in result.values():
                raise InputError(f"Invalid or duplicate {kind} identity: {name!r}")
            result[i] = name
        return result

    bn = identities("body", model.nbody)
    gn = identities("geom", model.ngeom)
    an = identities("actuator", model.nu)
    jn = identities("joint", model.njnt, required=False)
    bodies, actuators, geoms = {}, {}, {}
    for i, name in bn.items():
        r = rotation(model.body_iquat[i])
        bodies[name] = {
            "parent": bn[int(model.body_parentid[i])],
            "weld": bn[int(model.body_weldid[i])],
            "position": model.body_pos[i].tolist(),
            "rotation": rotation(model.body_quat[i]).tolist(),
            "mass": float(model.body_mass[i]),
            "com": model.body_ipos[i].tolist(),
            "inertia": (r @ np.diag(model.body_inertia[i]) @ r.T).tolist(),
            "contype": int(model.body_contype[i]),
            "conaffinity": int(model.body_conaffinity[i]),
        }
    for i, name in an.items():
        transmission = int(model.actuator_trntype[i])
        joint = int(model.actuator_trnid[i, 0])
        if transmission not in (0, 1) or joint not in jn:
            raise InputError(f"Actuator {name}: v0 requires a named joint transmission")
        dyntype = int(model.actuator_dyntype[i])
        if dyntype not in (0, 1, 2, 3):
            raise InputError(f"Actuator {name}: v0 supports none/integrator/filter/filterexact")
        actuators[name] = {
            "target": f"{transmission}:{jn[joint]}",
            "dyntype": dyntype,
            "dynprm": model.actuator_dynprm[i].tolist(),
            "actnum": int(model.actuator_actnum[i]),
            "actearly": bool(model.actuator_actearly[i]),
        }
    for i, name in gn.items():
        kind = int(model.geom_type[i])
        if kind == int(mujoco.mjtGeom.mjGEOM_SDF):
            raise InputError("SDF collision plugins are outside v0's collision checks")
        mesh_data = "not_applicable"
        if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
            mid = int(model.geom_dataid[i])
            mesh_data = (
                "present"
                if mid >= 0 and (model.mesh_polynum[mid] > 0 or model.mesh_graphadr[mid] >= 0)
                else "missing"
            )
        geoms[name] = {
            "body": bn[int(model.geom_bodyid[i])],
            "kind": kind,
            "contype": int(model.geom_contype[i]),
            "conaffinity": int(model.geom_conaffinity[i]),
            "group": int(model.geom_group[i]),
            "mesh_data": mesh_data,
        }
    flags = int(model.opt.disableflags)
    excluded = []
    for signature in model.exclude_signature:
        excluded.append([bn[int(signature) >> 16], bn[int(signature) & 0xFFFF]])
    result = {
        "format": "simdiff.mujoco",
        "schema_version": 1,
        "mujoco_version": mujoco.__version__,
        "label": label,
        "bodies": bodies,
        "actuators": actuators,
        "geoms": geoms,
        "collision": {
            "enabled": not bool(flags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)),
            "filter_parent": not bool(flags & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)),
            "excludes": sorted(sorted(pair) for pair in excluded),
            "explicit_pairs": sorted(
                sorted([gn[int(a)], gn[int(b)]])
                for a, b in zip(model.pair_geom1, model.pair_geom2, strict=True)
            ),
        },
    }
    if probe:
        scratch = mujoco.MjData(model)
        if data is not None:
            mujoco.mj_copyData(scratch, model, data)
        # Positions + collision detection only: no integration, controller or force solve.
        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_comPos(model, scratch)
        mujoco.mj_collision(model, scratch)
        result["probe"] = {
            "poses": {
                name: np.concatenate([scratch.xpos[i], scratch.xmat[i]]).tolist()
                for i, name in bn.items()
            },
            "contacts": [
                list(pair)
                for pair in sorted(
                    {
                        tuple(sorted([gn[int(c.geom1)], gn[int(c.geom2)]]))
                        for c in scratch.contact
                        if c.geom1 >= 0 and c.geom2 >= 0
                    }
                )
            ],
        }
    return Snapshot.model_validate(result)


def save(snapshot, path):
    """Write a validated snapshot with stable ordering and no NaN/Infinity."""
    snapshot = Snapshot.model_validate(snapshot)
    Path(path).write_text(
        json.dumps(snapshot.model_dump(), indent=2, sort_keys=True, allow_nan=False) + "\n"
    )


def load(path, *, probe=False):
    path = Path(path)
    if path.suffix.lower() == ".json":
        snapshot = Snapshot.model_validate(read_json(path))
        if probe and snapshot.probe is None:
            raise InputError(f"{path}: snapshot has no probe; capture it with probe=True")
        return snapshot
    if path.suffix.lower() not in (".xml", ".mjcf", ".urdf"):
        raise InputError("Expected MJCF (.xml/.mjcf), URDF, or a simdiff .json snapshot")
    if not path.is_file():
        raise InputError(f"Model file does not exist: {path}")
    return capture(mujoco.MjModel.from_xml_path(str(path.resolve())), label=str(path), probe=probe)
