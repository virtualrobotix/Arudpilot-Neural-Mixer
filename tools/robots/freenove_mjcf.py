#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Build the MuJoCo model of the Freenove Robot Dog (FNK0050) used for training and simulation.

    python tools/robots/freenove_mjcf.py            # writes robots/freenove/robot/freenove.xml and checks it
    python tools/robots/freenove_mjcf.py --render /tmp/freenove.png
    python tools/robots/freenove_mjcf.py --gait --video /tmp/freenove_gait.mp4   # upstream gait on the model

Upstream publishes no URDF, MJCF or 3D CAD. Geometry comes from the kinematic model the robot
runs (Code/Server/Control.py of Freenove_Robot_Dog_Kit_for_Raspberry_Pi):
  postureBalance()     l = 136 mm, b = 76 mm between the abduction axes
  coordinateToAngle()  l1 = 23 mm abduction -> hip pitch (vertical at neutral), l2 = l3 = 55 mm
  stop()               stand: foot at x = 10, y = 99 (down), z = +/-10 mm (outward) from each abduction axis
  Servo.py             servo clamp 18..162 deg, PCA9685 50 Hz
Joint zero = every servo at 90 deg (Tutorial.pdf, Step 15): thigh vertical, shank horizontal forward.
Visual geoms (group 2) follow the tutorial renders; collision geoms (group 3) are primitives that
carry the masses. Masses are estimates: weigh the assembled robot and edit MASS.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import robot_dir  # noqa: E402

# Control.py kinematics, metres
BODY_L, BODY_W = 0.136, 0.076
L1, L2, L3 = 0.023, 0.055, 0.055
STAND = (0.010, 0.099, 0.010)          # x forward, y down, z outward (left legs +z)
SERVO_LIMIT = math.radians(72.0)       # 18..162 deg around 90
FOOT_R = 0.006

# kg, about 0.55 in total (12 g servos, 2x 18650, Raspberry Pi 4B, shield, acrylic)
MASS = {
    "frame": 0.112,          # acrylic plates, brackets, screws
    "battery": 0.100,        # 2x 18650 + holder
    "electronics": 0.075,    # Raspberry Pi, shield, camera, LED module
    "abad_servo": 0.012,     # one per leg, fixed to the trunk
    "head": 0.035,           # S90 neck servo, ultrasonic module, acrylic
    "hip": 0.017,            # hip pitch servo + U bracket
    "thigh": 0.008,
    "knee_servo": 0.012,
    "shank": 0.007,
    "foot": 0.001,
}

# EMAX ES08MA II: 1.6 kgf.cm at 4.8 V, 2.0 at 6 V, 0.12 s/60 deg -> kv = stall torque / no-load speed
SERVO_TORQUE = 0.17
SERVO_KP = 2.0              # full torque at ~5 deg of error
SERVO_KV = 0.02
ARMATURE = 1.0e-4           # rotor inertia through the gearbox
JOINT_DAMPING = 0.002

LEGS = (("FL", +1, +1), ("FR", +1, -1), ("RL", -1, +1), ("RR", -1, -1))   # name, front, left


def freenove_ik(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Control.coordinateToAngle() without its rounding, returned in the joint convention of this model."""
    l1, l2, l3 = L1 * 1e3, L2 * 1e3, L3 * 1e3
    x, y, z = x * 1e3, y * 1e3, z * 1e3
    a = math.pi / 2 - math.atan2(z, y)
    l23 = math.sqrt((z - l1 * math.cos(a)) ** 2 + (y - l1 * math.sin(a)) ** 2 + x ** 2)
    b = math.asin(x / l23) - math.acos((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23))
    c = math.pi - math.acos((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2))
    return math.pi / 2 - a, -b, math.pi / 2 - c


def stand_q0() -> list[float]:
    q = []
    for _, _, left in LEGS:
        q += freenove_ik(STAND[0], STAND[1], STAND[2] * left)
    return q


def home_z() -> float:
    roll, pitch, knee = freenove_ik(*STAND)
    # lowest point of the foot sphere below the abduction axis, leg in its sagittal plane
    shank = pitch + knee
    depth = L1 + L2 * math.cos(pitch) + (L3 - FOOT_R) * math.sin(shank) + FOOT_R
    return round(depth * math.cos(roll), 4)


def _f(v) -> str:
    return " ".join(f"{x:.4g}" if abs(x) >= 1e-9 else "0" for x in v)


def leg_xml(name: str, front: int, left: int) -> str:
    x, y = front * BODY_L / 2, left * BODY_W / 2
    s = left
    lim = f"{-SERVO_LIMIT:.4f} {SERVO_LIMIT:.4f}"
    return f"""
      <body name="{name}_hip" pos="{_f((x, y, 0))}">
        <joint name="{name}_hip_roll" axis="1 0 0" range="{lim}"/>
        <geom class="mass" type="box" size="0.0115 0.006 0.012" pos="{_f((0, -s * 0.004, -0.019))}" mass="{MASS['hip']}"/>
        <geom class="vis_frame" type="box" size="0.013 0.0015 0.016" pos="{_f((0, s * 0.0035, -0.011))}"/>
        <geom class="vis_servo" type="box" size="0.0115 0.006 0.012" pos="{_f((0, -s * 0.004, -0.019))}"/>
        <geom class="vis_horn" type="cylinder" size="0.009 0.001" pos="{_f((0, s * 0.0055, -L1))}" zaxis="0 1 0"/>
        <body name="{name}_thigh" pos="{_f((0, 0, -L1))}">
          <joint name="{name}_hip_pitch" axis="0 1 0" range="{lim}"/>
          <geom class="collision" type="capsule" size="0.007" fromto="{_f((0, s * 0.007, -0.004, 0, s * 0.007, -0.046))}" mass="{MASS['thigh']}"/>
          <geom class="mass" type="box" size="0.012 0.006 0.0115" pos="{_f((0, -s * 0.0075, -0.045))}" mass="{MASS['knee_servo']}"/>
          <geom class="vis_frame" type="box" size="0.011 0.0015 0.0275" pos="{_f((0, s * 0.0075, -L2 / 2))}"/>
          <geom class="vis_frame" type="cylinder" size="0.011 0.0015" pos="{_f((0, s * 0.0075, 0))}" zaxis="0 1 0"/>
          <geom class="vis_frame" type="cylinder" size="0.011 0.0015" pos="{_f((0, s * 0.0075, -L2))}" zaxis="0 1 0"/>
          <geom class="vis_servo" type="box" size="0.012 0.006 0.0115" pos="{_f((0, -s * 0.0075, -0.045))}"/>
          <geom class="vis_label" type="box" size="0.003 0.0062 0.005" pos="{_f((0.006, -s * 0.0075, -0.035))}"/>
          <body name="{name}_shank" pos="{_f((0, 0, -L2))}">
            <joint name="{name}_knee" axis="0 1 0" range="{lim}"/>
            <geom class="collision" type="capsule" size="0.0045" fromto="{_f((0.008, 0, 0, L3 - FOOT_R - 0.004, 0, 0))}" mass="{MASS['shank']}"/>
            <geom name="{name}_foot" class="foot" pos="{_f((L3 - FOOT_R, 0, 0))}" mass="{MASS['foot']}"/>
            <geom class="vis_horn" type="cylinder" size="0.009 0.001" pos="{_f((0, s * 0.0035, 0))}" zaxis="0 1 0"/>
            <geom class="vis_frame" type="capsule" size="0.008 0.0015" fromto="{_f((0, 0, 0, 0.026, 0, -0.004))}"/>
            <geom class="vis_frame" type="capsule" size="0.0055 0.0015" fromto="{_f((0.026, 0, -0.004, L3 - FOOT_R, 0, 0))}"/>
            <geom class="vis_label" type="box" size="0.006 0.0017 0.002" pos="{_f((0.028, 0, -0.002))}"/>
            <site name="{name}_foot" pos="{_f((L3, 0, 0))}" size="0.003"/>
            <site name="{name}_touch" type="sphere" pos="{_f((L3 - FOOT_R, 0, 0))}" size="{FOOT_R + 0.0015}" group="4"/>
          </body>
        </body>
      </body>"""


def build_xml() -> str:
    q0 = stand_q0()
    z0 = home_z()
    joints = [f"{n}_{j}" for n, _, _ in LEGS for j in ("hip_roll", "hip_pitch", "knee")]
    legs = "".join(leg_xml(*leg) for leg in LEGS)
    actuators = "\n".join(f'    <position name="{j}" joint="{j}"/>' for j in joints)
    touch = "\n".join(f'    <touch name="{n}_touch" site="{n}_touch"/>' for n, _, _ in LEGS)
    abad = "\n".join(
        f'      <geom class="mass" type="box" size="0.012 0.0115 0.006" '
        f'pos="{_f((f * (BODY_L / 2 - 0.013), l * (BODY_W / 2 - 0.009), 0))}" mass="{MASS["abad_servo"]}"/>\n'
        f'      <geom class="vis_servo" type="box" size="0.012 0.0115 0.006" '
        f'pos="{_f((f * (BODY_L / 2 - 0.013), l * (BODY_W / 2 - 0.009), 0))}"/>\n'
        f'      <geom class="vis_horn" type="cylinder" size="0.009 0.001" '
        f'pos="{_f((f * (BODY_L / 2 - 0.0005), l * BODY_W / 2, 0))}" zaxis="1 0 0"/>'
        for _, f, l in LEGS)
    q0s = _f(q0)
    return f"""<?xml version="1.0" encoding="utf-8"?>
<!--
  Freenove Robot Dog Kit for Raspberry Pi (FNK0050), MuJoCo model for training and simulation.
  Generated by tools/robots/freenove_mjcf.py: edit that script, not this file.
  Kinematics of Code/Server/Control.py: body {BODY_L * 1e3:.0f} x {BODY_W * 1e3:.0f} mm between the abduction
  axes, legs {L1 * 1e3:.0f} + {L2 * 1e3:.0f} + {L3 * 1e3:.0f} mm.
  Joint zero = every servo at 90 deg: thigh vertical, shank horizontal forward.
    hip_roll   axis +x, positive moves the foot to +y (left)
    hip_pitch  axis +y, positive moves the foot backward
    knee       axis +y, positive rotates the shank down
  Range = servo clamp 18..162 deg. Masses are estimates, {sum_mass():.3f} kg in total.
  Actuators: EMAX ES08MA II, position servo kp {SERVO_KP} N.m/rad, kv {SERVO_KV}, torque +/-{SERVO_TORQUE} N.m.
  Groups: 2 visual, 3 collision (masses), 4 touch sites.
-->
<mujoco model="freenove_dog">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.005" integrator="implicitfast"/>

  <visual>
    <global offwidth="1920" offheight="1080" azimuth="135" elevation="-20"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
  </visual>

  <default>
    <joint armature="{ARMATURE}" damping="{JOINT_DAMPING}"/>
    <geom condim="3" friction="0.8 0.02 0.001"/>
    <position kp="{SERVO_KP}" kv="{SERVO_KV}" forcerange="{-SERVO_TORQUE} {SERVO_TORQUE}" ctrlrange="{-SERVO_LIMIT:.4f} {SERVO_LIMIT:.4f}"/>
    <site group="5"/>
    <default class="collision">
      <geom group="3" rgba="0.8 0.2 0.2 0.4"/>
    </default>
    <default class="mass">
      <geom group="3" contype="0" conaffinity="0" rgba="0.2 0.2 0.8 0.4"/>
    </default>
    <default class="foot">
      <geom type="sphere" size="{FOOT_R}" friction="0.9 0.02 0.001" group="3" rgba="0.8 0.2 0.2 0.4"/>
    </default>
    <default class="visual">
      <geom group="2" contype="0" conaffinity="0" density="0"/>
      <default class="vis_frame"><geom material="acrylic"/></default>
      <default class="vis_servo"><geom material="servo"/></default>
      <default class="vis_horn"><geom material="horn"/></default>
      <default class="vis_label"><geom material="orange"/></default>
      <default class="vis_pcb"><geom material="pcb"/></default>
      <default class="vis_metal"><geom material="metal"/></default>
      <default class="vis_battery"><geom material="battery"/></default>
    </default>
  </default>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.7 0.9" rgb2="0.15 0.2 0.3" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.82 0.82" rgb2="0.68 0.68 0.68" width="512" height="512" mark="edge" markrgb="0.6 0.6 0.6"/>
    <material name="floor_mat" texture="grid" texrepeat="20 20" reflectance="0.08"/>
    <material name="acrylic" rgba="0.18 0.18 0.2 1" specular="0.4" shininess="0.6"/>
    <material name="servo" rgba="0.08 0.08 0.08 1" specular="0.2"/>
    <material name="horn" rgba="0.55 0.55 0.58 1" specular="0.6"/>
    <material name="orange" rgba="0.95 0.45 0.08 1"/>
    <material name="pcb" rgba="0.1 0.45 0.2 1" specular="0.3"/>
    <material name="metal" rgba="0.75 0.75 0.78 1" specular="0.8" shininess="0.8"/>
    <material name="battery" rgba="0.15 0.35 0.75 1" specular="0.5"/>
    <material name="neck" rgba="0.1 0.3 0.85 1"/>
  </asset>

  <worldbody>
    <light name="sun" pos="0.6 0.6 2" dir="-0.3 -0.3 -1" directional="true" castshadow="true" diffuse="0.6 0.6 0.6"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="floor_mat"/>

    <!-- trunk frame on the four abduction axes, x forward, y left, z up -->
    <body name="trunk" pos="0 0 {z0}">
      <freejoint name="floating_base"/>
      <camera name="track" mode="trackcom" pos="0.26 -0.26 0.14" xyaxes="0.707 0.707 0 -0.25 0.25 0.935"/>
      <camera name="side" mode="trackcom" pos="0 -0.35 0.03" xyaxes="1 0 0 0 0 1"/>
      <camera name="front" mode="trackcom" pos="0.35 0 0.06" xyaxes="0 1 0 -0.15 0 1"/>
      <site name="imu" pos="0 0 0.02" size="0.005"/>

      <geom name="trunk" class="collision" type="box" size="0.075 0.028 0.022" pos="0 0 0.016" mass="{MASS['frame']}"/>
      <geom class="mass" type="box" size="0.035 0.02 0.01" pos="0 0 0.004" mass="{MASS['battery']}"/>
      <geom class="mass" type="box" size="0.0425 0.028 0.008" pos="0 0 0.034" mass="{MASS['electronics']}"/>
{abad}
      <!-- acrylic frame: bottom plate, top plate, side rails, standoffs -->
      <geom class="vis_frame" type="box" size="0.08 0.03 0.0015" pos="0 0 -0.008"/>
      <geom class="vis_frame" type="box" size="0.08 0.03 0.0015" pos="0 0 0.022"/>
      <geom class="vis_frame" type="box" size="0.08 0.0015 0.009" pos="0 0.0285 0.007"/>
      <geom class="vis_frame" type="box" size="0.08 0.0015 0.009" pos="0 -0.0285 0.007"/>
      <geom class="vis_metal" type="cylinder" size="0.0022 0.006" pos="0.04 0.022 0.03"/>
      <geom class="vis_metal" type="cylinder" size="0.0022 0.006" pos="-0.04 0.022 0.03"/>
      <geom class="vis_metal" type="cylinder" size="0.0022 0.006" pos="0.04 -0.022 0.03"/>
      <geom class="vis_metal" type="cylinder" size="0.0022 0.006" pos="-0.04 -0.022 0.03"/>
      <!-- 2x 18650 in the holder of the shield -->
      <geom class="vis_servo" type="box" size="0.036 0.02 0.009" pos="0 0 0.004"/>
      <geom class="vis_battery" type="cylinder" size="0.009 0.0325" pos="0 0.0095 0.006" zaxis="1 0 0"/>
      <geom class="vis_battery" type="cylinder" size="0.009 0.0325" pos="0 -0.0095 0.006" zaxis="1 0 0"/>
      <!-- Raspberry Pi with the robot shield -->
      <geom class="vis_pcb" type="box" size="0.0425 0.028 0.0008" pos="-0.005 0 0.036"/>
      <geom class="vis_pcb" type="box" size="0.0425 0.028 0.0008" pos="-0.005 0 0.047"/>
      <geom class="vis_metal" type="box" size="0.009 0.008 0.007" pos="-0.04 0.012 0.044"/>
      <geom class="vis_metal" type="box" size="0.009 0.0065 0.0065" pos="-0.04 -0.012 0.044"/>
      <geom class="vis_label" type="box" size="0.025 0.0012 0.004" pos="-0.005 0.024 0.052"/>
      <geom class="vis_pcb" type="box" size="0.012 0.014 0.0008" pos="0.05 0 0.024"/>

      <!-- head: S90 neck servo, ultrasonic "eyes", camera, ears -->
      <body name="head" pos="0.095 0 0.012">
        <geom class="collision" type="box" size="0.012 0.028 0.022" pos="0.008 0 0.012" mass="{MASS['head']}"/>
        <geom class="visual" type="box" size="0.011 0.006 0.0115" pos="-0.004 0 0.004" material="neck"/>
        <geom class="vis_frame" type="box" size="0.0015 0.028 0.02" pos="0.016 0 0.014"/>
        <geom class="vis_metal" type="cylinder" size="0.008 0.006" pos="0.023 0.013 0.018" zaxis="1 0 0"/>
        <geom class="vis_metal" type="cylinder" size="0.008 0.006" pos="0.023 -0.013 0.018" zaxis="1 0 0"/>
        <geom class="vis_servo" type="box" size="0.003 0.005 0.005" pos="0.019 0 0.001"/>
        <geom class="vis_frame" type="box" size="0.0015 0.006 0.009" pos="0.016 0.02 0.038" euler="0.5 0 0"/>
        <geom class="vis_frame" type="box" size="0.0015 0.006 0.009" pos="0.016 -0.02 0.038" euler="-0.5 0 0"/>
      </body>
{legs}
    </body>
  </worldbody>

  <actuator>
{actuators}
  </actuator>

  <sensor>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
{touch}
  </sensor>

  <keyframe>
    <key name="home" qpos="0 0 {z0} 1 0 0 0 {q0s}" ctrl="{q0s}"/>
  </keyframe>
</mujoco>
"""


def sum_mass() -> float:
    return (MASS["frame"] + MASS["battery"] + MASS["electronics"] + MASS["head"]
            + 4 * (MASS["abad_servo"] + MASS["hip"] + MASS["thigh"] + MASS["knee_servo"] + MASS["shank"]
                   + MASS["foot"]))


def check(path: Path) -> None:
    """Forward kinematics against Control.py, total mass, and a 3 s stand on the servos."""
    import mujoco
    import numpy as np

    from catalog import ROBOTS

    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    q0 = stand_q0()
    cat = ROBOTS["freenove"]["q0"]
    assert np.allclose(cat, q0, atol=1e-4), f"catalog q0 {cat} != model stand {np.round(q0, 4).tolist()}"
    assert ROBOTS["freenove"]["sim"]["home_z"] == home_z(), f"catalog sim.home_z != {home_z()}"
    for name, front, left in LEGS:
        hip = d.xpos[m.body(f"{name}_hip").id]
        foot = d.site_xpos[m.site(f"{name}_foot").id]
        err = (foot - hip) - np.array([STAND[0], left * STAND[2], -STAND[1]])
        assert np.abs(err).max() < 2e-4, f"{name}: foot off by {err * 1e3} mm"
    total = float(m.body_subtreemass[m.body("trunk").id])
    for _ in range(600):
        mujoco.mj_step(m, d)
    tilt = math.degrees(math.acos(min(1.0, 1 - 2 * float(d.qpos[4] ** 2 + d.qpos[5] ** 2))))
    touch = [float(d.sensor(f"{n}_touch").data[0]) for n, _, _ in LEGS]
    print(f"check: foot positions match Control.py; mass {total:.3f} kg; after 3 s z {d.qpos[2]:.4f} m, "
          f"tilt {tilt:.2f} deg, foot load {np.round(touch, 2).tolist()} N, "
          f"max servo torque {np.abs(d.actuator_force).max():.3f} N.m")


FREENOVE_LEG = ("FL", "RL", "RR", "FR")    # index of point[] in Control.py
GAIT_STEP_S = 0.015                          # one Control.run(): 12 PCA9685 writes over I2C


def freenove_points(move: str, i: int, h: float = 99.0) -> list[tuple[float, float, float]]:
    """point[] of Control.forWard() / turnLeft() at loop angle i (mm, Control.changeCoordinates)."""
    c, s = math.cos(math.radians(i)), math.sin(math.radians(i))
    c2, s2 = -c, -s
    if move == "forward":
        x1, y1, x2, y2 = 12 * c, min(6 * s + h, h), 12 * c2, min(6 * s2 + h, h)
        return [(x1 + 10, y1, 10), (x2 + 10, y2, 10), (x1 + 10, y1, -10), (x2 + 10, y2, -10)]
    x1, y1, x2, y2 = 3 * c, min(8 * s + h, h), 3 * c2, min(8 * s2 + h, h)
    p = [None] * 4
    for k in range(2):
        p[2 * k] = ((-1) ** (1 + k) * x1 + 10, y1, (-1) ** k * x1 + (-1) ** k * 10)
        p[1 + 2 * k] = ((-1) ** (1 + k) * x2 + 10, y2, (-1) ** (1 + k) * x2 + (-1) ** k * 10)
    return p


def run_upstream_gait(path: Path, seconds: float = 6.0, speed: int = 8, video: Path | None = None) -> None:
    """Replay the open-loop gait of Control.py (forWard, then turnLeft) through its own IK on the model."""
    import subprocess

    import mujoco
    import numpy as np

    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    act = {m.actuator(a).name: a for a in range(m.nu)}
    sub = int(round(GAIT_STEP_S / m.opt.timestep))
    proc = r = cam = None
    if video:
        r = mujoco.Renderer(m, 540, 960)
        proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                 "-s", "960x540", "-r", str(round(1 / GAIT_STEP_S)), "-i", "-",
                                 "-pix_fmt", "yuv420p", "-vcodec", "libx264", str(video)], stdin=subprocess.PIPE)
    report = []
    for move in ("forward", "turn"):
        x0, yaw0, t, max_tilt = d.qpos[0], 0.0, 0.0, 0.0
        yaw_prev, yaw_sum = None, 0.0
        while t < seconds / 2:
            angles = range(90, 451, speed) if move == "forward" else range(0, 361, speed)
            for i in angles:
                for leg, (x, y, z) in zip(FREENOVE_LEG, freenove_points(move, i)):
                    q = freenove_ik(x * 1e-3, y * 1e-3, z * 1e-3)
                    for j, v in zip(("hip_roll", "hip_pitch", "knee"), q):
                        d.ctrl[act[f"{leg}_{j}"]] = v
                for _ in range(sub):
                    mujoco.mj_step(m, d)
                w, qx, qy, qz = d.qpos[3:7]
                yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                if yaw_prev is not None:
                    yaw_sum += math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))
                yaw_prev = yaw
                max_tilt = max(max_tilt, math.degrees(math.acos(min(1.0, 1 - 2 * (qx * qx + qy * qy)))))
                t += GAIT_STEP_S
                if proc:
                    r.update_scene(d, camera="track")
                    proc.stdin.write(r.render().tobytes())
        report.append(f"{move}: {t:.1f} s, dx {100 * (d.qpos[0] - x0):+.1f} cm, "
                      f"yaw {math.degrees(yaw_sum):+.0f} deg, max tilt {max_tilt:.1f} deg")
    if proc:
        proc.stdin.close()
        proc.wait()
        print(f"wrote {video}")
    print("upstream gait on the model: " + "; ".join(report))


def render(path: Path, out: Path, camera: str) -> None:
    import mujoco
    from PIL import Image

    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 720, 1280)
    r.update_scene(d, camera=camera)
    Image.fromarray(r.render()).save(out)
    print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=robot_dir("freenove") / "robot" / "freenove.xml")
    ap.add_argument("--render", type=Path, help="PNG of the stand pose")
    ap.add_argument("--camera", default="track")
    ap.add_argument("--gait", action="store_true", help="replay the upstream open-loop gait (forward, turn)")
    ap.add_argument("--video", type=Path, help="mp4 of the upstream gait")
    args = ap.parse_args()
    args.out.write_text(build_xml())
    print(f"wrote {args.out} (stand q0 {[round(v, 4) for v in stand_q0()]}, home_z {home_z()})")
    check(args.out)
    if args.render:
        render(args.out, args.render, args.camera)
    if args.gait or args.video:
        run_upstream_gait(args.out, video=args.video)


if __name__ == "__main__":
    main()
