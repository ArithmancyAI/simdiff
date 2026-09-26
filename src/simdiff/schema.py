"""Versioned, validated snapshots and reports; never executable model adapters."""

from typing import Annotated, Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

Vec3 = Annotated[list[float], Field(min_length=3, max_length=3)]
Mat3 = Annotated[list[Vec3], Field(min_length=3, max_length=3)]
Pair = Annotated[list[str], Field(min_length=2, max_length=2)]
Name = Annotated[str, Field(min_length=1)]


class InputError(ValueError):
    """The selected checks cannot be completed on this input."""


class Record(BaseModel):
    model_config = ConfigDict(
        extra="forbid", strict=True, allow_inf_nan=False, revalidate_instances="always"
    )


class Body(Record):
    parent: str
    weld: str
    position: Vec3
    rotation: Mat3
    mass: Annotated[float, Field(ge=0)]
    com: Vec3
    inertia: Mat3
    contype: int
    conaffinity: int

    @model_validator(mode="after")
    def matrices(self):
        r, tensor = np.asarray(self.rotation), np.asarray(self.inertia)
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(
            np.linalg.det(r), 1, atol=1e-7
        ):
            raise ValueError("body rotation must be a proper rotation matrix")
        if not np.allclose(tensor, tensor.T, rtol=0, atol=1e-14):
            raise ValueError("inertia must be symmetric")
        if np.linalg.eigvalsh(tensor).min() < -1e-14:
            raise ValueError("inertia must be positive semidefinite")
        return self


class Actuator(Record):
    target: Name
    dyntype: Literal[0, 1, 2, 3]
    dynprm: Annotated[list[float], Field(min_length=10, max_length=10)]
    actnum: Annotated[int, Field(ge=0)]
    actearly: bool


class Geom(Record):
    body: Name
    kind: Literal[0, 1, 2, 3, 4, 5, 6, 7]
    contype: int
    conaffinity: int
    group: int
    mesh_data: Literal["present", "missing", "not_applicable"]


class Collision(Record):
    enabled: bool
    filter_parent: bool
    excludes: list[Pair]
    explicit_pairs: list[Pair]


class Probe(Record):
    """Contacts at the captured configuration, not a rollout or universal proof."""

    poses: dict[Name, Annotated[list[float], Field(min_length=12, max_length=12)]]
    contacts: list[Pair]


class Snapshot(Record):
    format: Literal["simdiff.mujoco"]
    schema_version: Literal[1]
    mujoco_version: Name
    label: Name
    bodies: dict[Name, Body]
    actuators: dict[Name, Actuator]
    geoms: dict[Name, Geom]
    collision: Collision
    probe: Probe | None = None

    @model_validator(mode="after")
    def references(self):
        if "world" not in self.bodies:
            raise ValueError("snapshot must contain the world body")
        for name, body in self.bodies.items():
            if body.parent not in self.bodies or body.weld not in self.bodies:
                raise ValueError(f"body {name}: unknown parent or weld body")
            visited, current = set(), name
            while current != "world":
                if current in visited:
                    raise ValueError("body parent graph has a cycle")
                visited.add(current)
                current = self.bodies[current].parent
                if current not in self.bodies:
                    raise ValueError("unknown parent body")
        if self.bodies["world"].parent != "world" or self.bodies["world"].weld != "world":
            raise ValueError("world must be its own parent and weld body")
        for geom in self.geoms.values():
            if geom.body not in self.bodies:
                raise ValueError("geom references an unknown body")
        for pairs, inventory in (
            (self.collision.excludes, self.bodies),
            (self.collision.explicit_pairs, self.geoms),
            (self.probe.contacts if self.probe else [], self.geoms),
        ):
            if any(a not in inventory or b not in inventory or a == b for a, b in pairs):
                raise ValueError("pair has unknown or identical entities")
        if self.probe and set(self.probe.poses) != set(self.bodies):
            raise ValueError("probe must provide poses for every body")
        return self


class Finding(Record):
    severity: Literal["info", "warn", "breaking"]
    code: str
    entity: str
    field: str
    before: Any
    after: Any
    why: str
    accepted: bool = False
    reason: str | None = None


class Acceptance(Record):
    code: Name
    entity: Name
    field: Name
    before: Any
    after: Any
    reason: Name


class Allowlist(Record):
    schema_version: Literal[1]
    changes: list[Acceptance]
