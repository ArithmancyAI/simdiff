import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
from pydantic import ValidationError

from simdiff import capture, compare, load, save
from simdiff.compare import eligible_pairs, exit_code
from simdiff.schema import InputError, Snapshot

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "fixtures"


def model(name="good"):
    return mujoco.MjModel.from_xml_path(str(FIXTURES / f"{name}.xml"))


def codes(report):
    return {f["code"] for f in report["findings"]}


def cli(*args):
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    return subprocess.run(
        [sys.executable, "-m", "simdiff", *map(str, args)],
        capture_output=True,
        text=True,
        cwd=ROOT,
        env=env,
        check=False,
    )


def test_unchanged_model_passes_all_checks():
    report = compare(capture(model()), capture(model()), probe=False)
    assert report["status"] == "no_findings"
    assert report["findings"] == []
    assert report["coverage"]["actuators_compared"] == 9
    assert exit_code(report) == 0


def test_nine_filters_removed_fixture():
    report = compare(capture(model()), capture(model("filters_removed")))
    lost = [f for f in report["findings"] if f["code"] == "actuator.dyntype"]
    assert len(lost) == 9
    assert {f["entity"] for f in lost} == {f"actuator:drive_{i}" for i in range(9)}
    assert all(f["before"] == 3 and f["after"] == 0 and f["severity"] == "breaking" for f in lost)
    assert exit_code(report) == 1


def test_small_real_inertia_change_is_not_swallowed_by_tolerance():
    report = compare(capture(model()), capture(model("inertia_changed")))
    assert codes(report) == {"inertia.inertia"}
    finding = report["findings"][0]
    assert finding["entity"] == "body:payload"
    assert finding["severity"] == "breaking"


def test_collision_mutation_changes_pair_eligibility():
    report = compare(capture(model()), capture(model("collisions_disabled")))
    found = [f for f in report["findings"] if f["code"] == "collision.pair_eligibility"]
    assert len(found) == 1
    assert found[0]["before"] is True and found[0]["after"] is False
    assert found[0]["severity"] == "breaking"


def test_runtime_bug_equal_masks_different_compiled_state_and_contacts(tmp_path):
    before, after = model("mesh_enabled"), model("mesh_disabled")
    # Runtime-only edit: both geom arrays now match, but compiled state does not.
    after.geom_contype[:] = before.geom_contype
    after.geom_conaffinity[:] = before.geom_conaffinity
    np.testing.assert_array_equal(before.geom_contype, after.geom_contype)
    np.testing.assert_array_equal(before.geom_conaffinity, after.geom_conaffinity)
    assert before.mesh_graphadr[0] >= 0 and after.mesh_graphadr[0] < 0
    old, new = capture(before, probe=True), capture(after, probe=True)
    assert old.probe.contacts == [["floor", "shell"]]
    assert new.probe.contacts == []
    save(new, tmp_path / "runtime.json")
    report = compare(old, load(tmp_path / "runtime.json"), probe=True)
    assert "collision.body_mask_blocks_pair" in codes(report)
    assert "collision.mesh_data_missing" in codes(report)
    assert "collision.probe_contact" in codes(report)
    assert not {"collision.contype", "collision.conaffinity"} & codes(report)
    assert exit_code(report) == 1


def test_reordered_bodies_geoms_and_actuators_do_not_create_differences():
    root = ET.parse(FIXTURES / "good.xml").getroot()
    for tag in ("worldbody", "actuator"):
        element = root.find(tag)
        element[:] = list(reversed(list(element)))
    reordered = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert compare(capture(model()), capture(reordered))["findings"] == []


def test_equivalent_principal_axis_permutation_passes():
    before, after = model(), model()
    i = after.body("payload").id
    before.body_inertia[i] = [1e-7, 2e-7, 3e-7]
    after.body_inertia[i] = [2e-7, 1e-7, 3e-7]
    after.body_iquat[i] = [2**-0.5, 0, 0, 2**-0.5]
    assert compare(capture(before), capture(after))["findings"] == []


def test_quaternion_sign_is_equivalent():
    a, b = model(), model()
    b.body_quat[:] *= -1
    b.body_iquat[:] *= -1
    assert compare(capture(a), capture(b))["findings"] == []


@pytest.mark.parametrize(
    "field,value,expected",
    [
        ("body_mass", 0.02, "inertia.mass"),
        ("body_ipos", [0.001, 0, 0], "inertia.com"),
    ],
)
def test_mass_and_com(field, value, expected):
    a, b = model(), model()
    getattr(b, field)[b.body("payload").id] = value
    assert codes(compare(capture(a), capture(b))) == {expected}


def test_filter_time_constant_and_unused_parameters():
    a, b = model(), model()
    b.actuator_dynprm[:, 1:] = 99
    assert compare(capture(a), capture(b))["findings"] == []
    b.actuator_dynprm[0, 0] = 0.5
    assert codes(compare(capture(a), capture(b))) == {"actuator.filter_time_constant"}


def test_unused_none_dynamics_parameters_do_not_fail():
    a, b = model("filters_removed"), model("filters_removed")
    b.actuator_dynprm[:] = 200
    assert compare(capture(a), capture(b))["findings"] == []


def test_changed_actuator_joint_is_not_silently_matched_by_name():
    a, b = model(), model()
    b.actuator_trnid[0, 0] = b.joint("joint_1").id
    assert "actuator.target" in codes(compare(capture(a), capture(b)))


def test_missing_actuator_reports_inventory_and_count():
    root = ET.parse(FIXTURES / "good.xml").getroot()
    root.find("actuator").remove(root.find("actuator")[0])
    b = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    report = compare(capture(model()), capture(b))
    assert {"actuator.removed", "actuator.count"} <= codes(report)


def test_geom_group_is_info_not_collision_filter():
    a, b = model(), model()
    b.geom_group[:] = 3
    report = compare(capture(a), capture(b))
    assert codes(report) == {"collision.group"}
    assert all(f["severity"] == "info" for f in report["findings"])
    assert exit_code(report) == 0 and exit_code(report, "info") == 1


def test_redundant_masks_are_warn_not_breaking():
    a, b = model(), model()
    b.geom_contype[:] *= 3
    b.geom_conaffinity[:] *= 3
    b.body_contype[:] *= 3
    b.body_conaffinity[:] *= 3
    report = compare(capture(a), capture(b))
    assert "collision.pair_eligibility" not in codes(report)
    assert exit_code(report) == 0 and exit_code(report, "warn") == 1


def contact_model(extra="", option=""):
    return mujoco.MjModel.from_xml_string(f"""<mujoco>{option}<worldbody>
      <geom name="floor" type="plane" size="1 1 0.1"/>
      <body name="a" pos="0 0 .015"><freejoint/>
        <geom name="ga" size=".02"/>
        <body name="child" pos=".02 0 0"><joint type="slide"/>
          <geom name="gc" size=".02"/>
        </body>
        <body name="welded"><geom name="gw" size=".01"/></body>
      </body>
      <body name="b" pos=".03 0 .015"><freejoint/><geom name="gb" size=".02"/></body>
      </worldbody>{extra}</mujoco>""")


def test_collision_parent_weld_and_world_rules():
    snapshot = capture(contact_model())
    pairs = eligible_pairs(snapshot)
    assert ("ga", "gc") not in pairs
    assert ("ga", "gw") not in pairs
    assert ("gc", "gw") not in pairs
    assert ("floor", "ga") in pairs
    assert ("ga", "gb") in pairs


def test_explicit_pair_bypasses_parent_filter_and_masks():
    m = contact_model('<contact><pair geom1="ga" geom2="gc"/></contact>')
    m.geom_contype[:] = 0
    m.geom_conaffinity[:] = 0
    assert eligible_pairs(capture(m)) == {("ga", "gc")}


def test_excludes_remove_dynamic_pairs_but_not_explicit_pairs():
    m = contact_model('<contact><exclude body1="a" body2="b"/></contact>')
    assert ("ga", "gb") not in eligible_pairs(capture(m))
    m = contact_model(
        '<contact><exclude body1="a" body2="b"/><pair geom1="ga" geom2="gb"/></contact>'
    )
    assert ("ga", "gb") in eligible_pairs(capture(m))


def test_disable_parent_filter_and_global_contacts():
    m = contact_model(option='<option><flag filterparent="disable"/></option>')
    assert ("ga", "gc") in eligible_pairs(capture(m))
    m = contact_model(option='<option><flag contact="disable"/></option>')
    assert eligible_pairs(capture(m)) == set()


def test_probe_does_not_mutate_caller_data():
    m = model("mesh_enabled")
    data = mujoco.MjData(m)
    data.qpos[2] = 0.019
    data.time = 4
    data.qvel[:] = 2
    original = (data.qpos.copy(), data.qvel.copy(), data.time)
    snapshot = capture(m, probe=True, data=data)
    assert snapshot.probe.contacts
    np.testing.assert_array_equal(data.qpos, original[0])
    np.testing.assert_array_equal(data.qvel, original[1])
    assert data.time == original[2]


def test_different_probe_poses_are_incomplete():
    m = model("mesh_enabled")
    data = mujoco.MjData(m)
    data.qpos[2] = 0.5
    with pytest.raises(InputError, match="different body poses"):
        compare(capture(m, probe=True), capture(m, probe=True, data=data), probe=True)


def test_missing_probe_is_not_a_pass():
    with pytest.raises(InputError, match="requires probes"):
        compare(capture(model()), capture(model()), probe=True)


def test_body_frame_changes_are_explicitly_unsupported():
    a, b = model(), model()
    b.body_pos[b.body("payload").id, 0] += 0.1
    with pytest.raises(InputError, match="frame changed"):
        compare(capture(a), capture(b))


def test_unnamed_entity_requires_explicit_identity_and_override_works():
    m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=".1"/></worldbody></mujoco>')
    with pytest.raises(InputError, match="Unnamed geom"):
        capture(m)
    assert set(capture(m, names={"geom": {0: "ball"}}).geoms) == {"ball"}
    with pytest.raises(InputError, match="Invalid geom index"):
        capture(m, names={"geom": {1: "ball"}})


def test_duplicate_override_names_are_rejected():
    with pytest.raises(InputError, match="duplicate"):
        capture(model(), names={"actuator": {0: "drive_1"}})


def test_exact_allowlist_preserves_findings_and_does_not_hide_new_values():
    before, after = capture(model()), capture(model("inertia_changed"))
    report = compare(before, after)
    f = report["findings"][0]
    rule = {k: f[k] for k in ("code", "entity", "field", "before", "after")}
    rule["reason"] = "Approved inertia recomputation for this payload."
    allowlist = {"schema_version": 1, "changes": [rule]}
    accepted = compare(before, after, allowlist=allowlist)
    assert accepted["status"] == "accepted_changes"
    assert accepted["findings"][0]["accepted"] is True
    assert exit_code(accepted) == 0
    mutated = model("inertia_changed")
    mutated.body_inertia[mutated.body("payload").id] *= 2
    with pytest.raises(InputError, match="unmatched"):
        compare(before, capture(mutated), allowlist=allowlist)


@pytest.mark.parametrize("rtol,atol", [(float("nan"), 0), (-1, 0), (0, float("inf"))])
def test_invalid_tolerances(rtol, atol):
    with pytest.raises(InputError, match="Tolerances"):
        compare(capture(model()), capture(model()), rtol=rtol, atol=atol)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda s: s.update(schema_version=2),
        lambda s: s.pop("schema_version"),
        lambda s: s.pop("format"),
        lambda s: s["bodies"]["payload"].update(mass=float("nan")),
        lambda s: s["bodies"]["payload"].update(inertia=[[1, 2, 0], [0, 1, 0], [0, 0, 1]]),
        lambda s: s["geoms"]["payload_geom"].update(body="missing"),
        lambda s: s["bodies"]["payload"].update(parent="payload"),
        lambda s: s["bodies"].pop("world"),
        lambda s: s["actuators"]["drive_0"].update(dyntype=999),
    ],
)
def test_invalid_snapshot_is_rejected(mutation):
    document = capture(model()).model_dump()
    mutation(document)
    with pytest.raises(ValidationError):
        Snapshot.model_validate(document)


def test_duplicate_json_keys_rejected(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text('{"schema_version":1,"schema_version":1}')
    with pytest.raises(InputError, match="Duplicate"):
        load(path)


def test_snapshot_roundtrip(tmp_path):
    original = capture(model(), probe=True)
    save(original, tmp_path / "model.json")
    assert load(tmp_path / "model.json") == original


@pytest.mark.parametrize(
    "fixture,expected",
    [
        ("good", 0),
        ("filters_removed", 1),
        ("inertia_changed", 1),
        ("collisions_disabled", 1),
    ],
)
def test_cli_machine_readable_exit_codes(fixture, expected):
    result = cli(FIXTURES / "good.xml", FIXTURES / f"{fixture}.xml", "--format", "json")
    assert result.returncode == expected, result.stderr
    assert json.loads(result.stdout)["schema_version"] == 1
    assert result.stderr == ""


def test_cli_fail_on_none_keeps_findings():
    result = cli(
        FIXTURES / "good.xml",
        FIXTURES / "filters_removed.xml",
        "--format",
        "json",
        "--fail-on",
        "none",
    )
    assert result.returncode == 0
    assert json.loads(result.stdout)["summary"]["breaking"] == 18


@pytest.mark.parametrize("filename,contents", [("bad.xml", "<broken>"), ("bad.json", "{}")])
def test_cli_invalid_input_is_error_two(tmp_path, filename, contents):
    path = tmp_path / filename
    path.write_text(contents)
    result = cli(FIXTURES / "good.xml", path, "--format", "json")
    assert result.returncode == 2
    assert json.loads(result.stdout)["status"] == "error"


def test_cli_snapshot_and_probe(tmp_path):
    path = tmp_path / "saved.json"
    result = cli("snapshot", FIXTURES / "mesh_enabled.xml", "--probe", "-o", path)
    assert result.returncode == 0, result.stderr
    result = cli(FIXTURES / "mesh_enabled.xml", path, "--probe", "--format", "json")
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["coverage"]["contact_probe"] is True


def test_cli_table_contains_values_reason_and_scope():
    result = cli(FIXTURES / "good.xml", FIXTURES / "inertia_changed.xml")
    assert result.returncode == 1
    assert all(s in result.stdout for s in ("breaking", "body:payload", "kg m²", "Scope:"))


def test_urdf_compiles_and_reports_inertia(tmp_path):
    text = """<robot name="simple"><mujoco><compiler fusestatic="false"/></mujoco>
      <link name="base"/>
      <link name="payload"><inertial><mass value=".1"/>
        <inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/>
      </inertial><collision name="payload_geom"><geometry><sphere radius=".02"/></geometry>
      </collision></link>
      <joint name="hinge" type="continuous"><parent link="base"/><child link="payload"/>
        <axis xyz="0 0 1"/></joint></robot>"""
    a, b = tmp_path / "a.urdf", tmp_path / "b.urdf"
    a.write_text(text)
    b.write_text(text.replace('mass value=".1"', 'mass value=".2"'))
    assert codes(compare(load(a), load(b))) == {"inertia.mass"}


def test_loaded_snapshot_is_revalidated_after_mutation():
    snapshot = capture(model())
    snapshot.bodies["payload"].mass = float("nan")
    with pytest.raises(ValidationError):
        compare(capture(model()), snapshot)


def test_compiler_versions_are_reported_as_warning():
    a, b = capture(model()), capture(model())
    b.mujoco_version = "3.9.0" if a.mujoco_version != "3.9.0" else "3.14.0"
    report = compare(a, b)
    assert codes(report) == {"input.compiler_version"}
    assert exit_code(report) == 0 and exit_code(report, "warn") == 1


def test_missing_file_is_error_even_with_fail_on_none(tmp_path):
    result = cli(
        FIXTURES / "good.xml", tmp_path / "missing.xml", "--format", "json", "--fail-on", "none"
    )
    assert result.returncode == 2
    assert json.loads(result.stdout)["status"] == "error"
