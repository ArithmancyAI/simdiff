"""Rebuild original, synthetic fixtures. No robot assets or private data are used."""

from math import cos, pi, sin
from pathlib import Path

ROOT = Path(__file__).parent


def build():
    bodies = []
    actuators = []
    for i in range(9):
        bodies.append(f'''    <body name="link_{i}" pos="{i * 0.1:.1f} 0.2 0.1">
      <joint name="joint_{i}" type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
      <geom name="link_geom_{i}" type="box" size="0.01 0.01 0.01" contype="0" conaffinity="0"/>
    </body>''')
        actuators.append(
            f'    <general name="drive_{i}" joint="joint_{i}" dyntype="filterexact" dynprm="0.25"/>'
        )
    good = (
        """<mujoco model="simdiff_synthetic">
  <compiler angle="radian"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="payload" pos="0 0 0.025">
      <freejoint name="payload_joint"/>
      <inertial pos="0 0 0" mass="0.01" diaginertia="1e-7 1e-7 1e-7"/>
      <geom name="payload_geom" type="box" size="0.01 0.01 0.01"/>
    </body>
"""
        + "\n".join(bodies)
        + "\n  </worldbody>\n  <actuator>\n"
        + "\n".join(actuators)
        + """
  </actuator>
</mujoco>
"""
    )
    (ROOT / "good.xml").write_text(good)
    (ROOT / "filters_removed.xml").write_text(
        good.replace('dyntype="filterexact"', 'dyntype="none"')
    )
    (ROOT / "inertia_changed.xml").write_text(
        good.replace("1e-7 1e-7 1e-7", "4.46e-8 4.46e-8 5.73e-8")
    )
    (ROOT / "collisions_disabled.xml").write_text(
        good.replace('name="payload_geom"', 'name="payload_geom" contype="0" conaffinity="0"')
    )

    # A tessellated sphere exercises MuJoCo's compile-time mesh preprocessing.
    vertices = [(0, 0, 0.02), (0, 0, -0.02)]
    for latitude in range(1, 12):
        for longitude in range(24):
            theta, phi = pi * latitude / 12, 2 * pi * longitude / 24
            vertices.append(
                (0.02 * sin(theta) * cos(phi), 0.02 * sin(theta) * sin(phi), 0.02 * cos(theta))
            )
    mesh = " ".join(f"{v:.10g}" for xyz in vertices for v in xyz)
    faces = []
    for j in range(24):
        k = (j + 1) % 24
        faces.extend([(0, 2 + j, 2 + k), (1, 242 + k, 242 + j)])
        for ring in range(10):
            a, b = 2 + ring * 24, 2 + (ring + 1) * 24
            faces.extend([(a + j, b + j, b + k), (a + j, b + k, a + k)])
    triangles = " ".join(str(v) for face in faces for v in face)
    for enabled in (False, True):
        text = f'''<mujoco model="compile_time_collision_fixture">
  <asset><mesh name="sphere_mesh" vertex="{mesh}" face="{triangles}"/></asset>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="shell_body" pos="0 0 0.015">
      <freejoint name="shell_joint"/>
      <inertial pos="0 0 0" mass="0.01" diaginertia="1e-6 1e-6 1e-6"/>
      <geom name="shell" type="mesh" mesh="sphere_mesh"
            contype="{int(enabled)}" conaffinity="{int(enabled)}"/>
    </body>
  </worldbody>
</mujoco>
'''
        (ROOT / f"mesh_{'enabled' if enabled else 'disabled'}.xml").write_text(text)


if __name__ == "__main__":
    build()
