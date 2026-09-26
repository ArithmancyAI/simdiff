import mujoco
import numpy as np
import pytest

from simdiff import capture, compare
from simdiff.schema import InputError


def arm():
    return mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <geom name="floor" type="plane" size="1 1 .1"/>
      <body name="arm" pos=".2 -.1 .035" quat=".8 .2 .4 .4">
        <joint name="hinge" type="hinge"/>
        <inertial pos="0 0 .02" mass=".01" diaginertia="1e-7 1e-7 1e-7"/>
        <geom name="shape" type="sphere" size=".01"/>
      </body></worldbody><actuator>
        <general name="drive" joint="hinge" dyntype="filterexact" dynprm=".25"/>
      </actuator></mujoco>""")


def roundtrip(model):
    for field in ("body_pos", "body_quat", "body_iquat", "body_mass", "body_ipos", "body_inertia"):
        value = getattr(model, field)
        value[:] = value.astype(np.float32).astype(np.float64)


@pytest.mark.parametrize("probe", [False, True])
def test_float32_roundtrip_passes_without_relaxing_inertia_tolerance(probe):
    before, after = arm(), arm()
    roundtrip(after)
    report = compare(capture(before, probe=probe), capture(after, probe=probe), probe=probe)
    assert report["findings"] == []
    assert report["tolerance"]["atol"] == 1e-12
    assert report["frame_tolerance"]["position_atol_m"] == 1e-7


def test_float32_roundtrip_preserves_detection_of_small_inertia_change():
    before, after = arm(), arm()
    roundtrip(after)
    after.body_inertia[1] *= 0.5
    report = compare(capture(before), capture(after))
    assert [f["code"] for f in report["findings"]] == ["inertia.inertia"]


@pytest.mark.parametrize(
    "field,change",
    [
        ("body_pos", [1e-5, 0, 0]),
        ("body_pos", [0, 0, 1e-5]),
    ],
)
def test_actual_frame_translation_is_rejected_even_with_loose_physics_tolerance(field, change):
    before, after = arm(), arm()
    getattr(after, field)[1] += change
    with pytest.raises(InputError, match="frame changed"):
        compare(capture(before), capture(after), rtol=1, atol=1)


def test_actual_frame_rotation_is_rejected():
    before, after = arm(), arm()
    after.body_quat[1] = [1, 0, 0, 0]
    with pytest.raises(InputError, match="frame changed"):
        compare(capture(before), capture(after))


def test_translation_away_from_zero_has_only_the_absolute_allowance():
    before, after = arm(), arm()
    before.body_pos[1, 0] = 0
    after.body_pos[1, 0] = 1e-6
    with pytest.raises(InputError, match="frame changed"):
        compare(capture(before), capture(after))


def test_invalid_quaternion_is_not_normalized_into_a_pass():
    model = arm()
    model.body_quat[1] = [2, 0, 0, 0]
    with pytest.raises(InputError, match="unit length"):
        capture(model)


def test_tensor_rotation_roundoff_is_scaled_to_the_tensor_not_a_zero_component():
    before, after = arm(), arm()
    before.body_inertia[1] = [2e-4, 5e-5, 1e-5]
    after.body_inertia[1] = before.body_inertia[1]
    before.body_iquat[1] = [1, 0, 0, 0]
    after.body_iquat[1] = [np.cos(5e-8), np.sin(5e-8), 0, 0]
    assert compare(capture(before), capture(after))["findings"] == []
    # Increasing the change beyond the same relative tolerance still fails.
    after.body_inertia[1, 0] *= 1.01
    assert compare(capture(before), capture(after))["summary"]["breaking"] == 1


def test_importer_transform_cancellation_has_a_small_absolute_allowance():
    before, after = arm(), arm()
    before.body_pos[1] = [0.0004, 0.017, 0.010]
    after.body_pos[1] = [0.0004000001, 0.01700005, 0.010000005]
    assert compare(capture(before), capture(after))["findings"] == []
