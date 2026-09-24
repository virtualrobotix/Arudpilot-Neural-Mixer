# MicroDuck on ArduPilot — a PPO locomotion policy running as an ArduRover task, MuJoCo as the SITL plant

**English** | [Italiano](README.it.md)

*Developed by Roberto Navoni — DelphyAI LAB · r.navoni74@gmail.com*

This repository puts a reinforcement-learning walking controller for the [MicroDuck](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
(a 25 cm, 14-servo biped by Pollen Robotics / Hugging Face) **inside ArduPilot**. The neural network runs as a
50 Hz task of the ArduRover firmware, reads only ArduPilot sensors, and drives the 14 joints through ArduPilot's
servo outputs. In simulation the robot body is a MuJoCo model connected to ArduPilot's SITL exactly like a
physics engine (Gazebo, RealFlight) would be. You command it from MAVProxy with sticks, arm/disarm and flight modes.

Nothing here assumes prior knowledge of RL or of ArduPilot internals: the [glossary](#glossary) at the end
defines every term used.

```
 MAVProxy / GCS  ──MAVLink (tcp:5760)──▶  ArduRover SITL (one binary)                 ──SIM_JSON udp:9002──▶  MuJoCo plant
   arm, rc sticks,                          RC_Channels ─▶ AP_MicroDuck (PPO, 50 Hz) ─▶ SRV_Channels (14 PWM)      14 XL330 (BAM model)
   mode HOLD, params                        AP_InertialSensor ◀── SITL JSON backend ◀────udp:9003 (imu, quat, joints)────┘
```

Demo video (68 s, MAVProxy script → ArduPilot → network → MuJoCo, with the MAVLink commands shown on screen):
[`docs/media/demo_mavproxy_mlp.mp4`](docs/media/demo_mavproxy_mlp.mp4).
Architecture slides: **EN** [`docs/architecture-integration.pdf`](docs/architecture-integration.pdf) / [`.pptx`](docs/architecture-integration.pptx) ·
IT [`docs/architettura-integrazione.pdf`](docs/architettura-integrazione.pdf) — both built by `tools/build_architecture_slides.py`.

---

## Contents

1. [What problem this solves](#1-what-problem-this-solves)
2. [How the network was trained (the `.pt` file)](#2-how-the-network-was-trained-the-pt-file)
3. [The two networks: MLP and Cartan](#3-the-two-networks-mlp-and-cartan)
4. [From `.pt` to C code in the firmware](#4-from-pt-to-c-code-in-the-firmware)
5. [Architecture of the integration](#5-architecture-of-the-integration)
6. [What was changed in ArduPilot](#6-what-was-changed-in-ardupilot)
7. [Setup](#7-setup)
8. [Running the demo and the tests](#8-running-the-demo-and-the-tests)
9. [Results](#9-results)
10. [Repository layout](#10-repository-layout)
11. [Glossary](#glossary)

---

## 1. What problem this solves

A classical autopilot controls a wheeled rover with PID loops on speed and heading. A biped cannot be balanced
that way: the mapping from sensors to the 14 servo positions that keeps the robot upright while walking is
learned, not written by hand. The learned controller is a neural network called a **policy**.

The question this repo answers: *can that policy live inside ArduPilot, using ArduPilot's own sensor,
RC and servo stack, so that the same firmware runs first in simulation and later on the flight
controller of the real robot?* The answer is yes, and the repository is the working proof.

Three principles were kept throughout:

- **One firmware.** No companion computer, no second autopilot. The policy is a scheduler task of ArduRover.
- **Only ArduPilot APIs.** The task reads `AP_InertialSensor` (IMU), `RC_Channels` (sticks), a joint-feedback
  source, and writes `SRV_Channels`. It does not know that MuJoCo exists. Swapping the simulator for the real
  IMU and servo bus changes nothing in the task.
- **The network is plain C.** No ONNX Runtime, no Python in the firmware. The weights are `const float` arrays,
  the forward pass is ~100 lines of C, identical on the Mac (SITL) and on an STM32.

---

## 2. How the network was trained (the `.pt` file)

The training itself was **not** done in this repository. It was done with the official Pollen Robotics stack
([`pollen-robotics/microduck_rl`](https://github.com/pollen-robotics/microduck_rl)) and documented in the
NOESIS EXPERIMENT lab notebooks. This section explains it because the firmware only makes sense if you know
what the network expects.

### 2.1 Reinforcement learning in one paragraph

The robot is simulated thousands of times in parallel on a GPU. At every control step (50 times per second)
the policy network receives a vector of numbers describing the robot's state (the **observation**), outputs a
vector of 14 numbers (the **action**: where each servo should go), the simulator advances 20 ms, and a
**reward** is computed: positive for walking at the commanded speed and staying upright, negative for falling,
jerky actions, joints at their limits. The learning algorithm, **PPO** (Proximal Policy Optimization), adjusts
the network weights so that actions leading to higher long-term reward become more likely. After enough
iterations the network has "discovered" a gait. Nobody programmed the gait.

### 2.2 The simulator used for training

| Item | Value |
|---|---|
| Physics | **MuJoCo Warp** (GPU MuJoCo) via `mjlab` 1.3.0 |
| Robot model | MicroDuck MJCF from Onshape (14 hinge joints, trunk free joint) |
| Actuators | **BAM M6** model of the Dynamixel **XL330** servo: voltage-controlled, with measured friction, current limit, battery sag |
| Physics step | 5 ms (200 Hz); the policy acts every 4 steps → **50 Hz** |
| Parallel environments | **2048** robots simulated at once |
| Episode | 20 s (1000 control steps), ends early if the robot falls |
| Hardware | one NVIDIA RTX 3090 (Vast.ai) |

### 2.3 What the network sees and does — the contract

This is the interface that the firmware must reproduce **exactly**. The policy trained on these 61 numbers
in this order and these units; anything else is a different world for the network.

| Index | Size | Quantity | Unit | Frame |
|---|---:|---|---|---|
| `[0:3]` | 3 | trunk angular velocity (gyro) | rad/s | trunk FLU (x forward, y left, z up) |
| `[3:6]` | 3 | gravity direction | unit vector (standing ≈ `[0,0,-1]`) | trunk FLU |
| `[6:20]` | 14 | joint position minus default pose `q0` | rad | joint order below |
| `[20:34]` | 14 | joint velocity | rad/s | |
| `[34:48]` | 14 | previous action (what was sent to the servos last step) | rad | |
| `[48:51]` | 3 | commanded twist `vx, vy` (m/s), `ωz` (rad/s) | SI | |
| `[51:55]` | 4 | head pose command (neck/head pitch, yaw, roll) | rad | 0 in this repo |
| `[55:61]` | 6 | body pose command (x, y, z, roll, pitch, yaw) | m, rad | 0 in this repo |

Action: 14 offsets in radians; the servo target is `q_target = q0 + action`.
Joint order: `left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle, neck_pitch, head_pitch,
head_yaw, head_roll, right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle`.
Default pose `q0` (rad): `[0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0, 0, 0, 0.0873, 0.4579, 0.0049, -0.4530]`.

Command ranges seen in training: `vx ∈ [-0.4, 0.4]` m/s, `vy ∈ [-0.3, 0.3]` m/s, `ωz ∈ [-1, 1]` rad/s.
Zero command = stand still (explicitly trained).

### 2.4 The reward (what "good" means)

Weights from the training configuration (`params/env.yaml` of the checkpoints):

| Term | Weight | Meaning |
|---|---:|---|
| `track_linear_velocity` | +2.0 | follow the commanded `vx, vy` |
| `track_angular_velocity` | +2.0 | follow the commanded `ωz` |
| `upright` | +2.0 | keep the trunk vertical |
| `head_pose_tracking` | +2.0 | keep the head where commanded |
| `pose` | +1.0 | stay near the default posture |
| `dof_pos_limits` | −1.0 | do not push joints to their mechanical limits |
| `action_rate_l2` | 0 → −0.8 (curriculum) | smooth actions, introduced progressively |
| `body_ang_vel`, `angular_momentum` | −0.05, −0.02 | do not thrash |

Termination: the episode ends when the trunk falls over (`fell_over`). At the end of training about half the
episodes still terminate by a fall within 20 s under the hardest randomizations — this is normal for a biped
this size and is why speed tracking is imperfect.

### 2.5 Domain randomization (why the policy survives a different simulator)

Every episode randomizes: foot friction, battery voltage (6.5–8.2 V), servo friction budget, joint armature,
mass/inertia and centre of mass of trunk and head, encoder bias, IMU misalignment, sensor noise and a 0–1 step
delay on the gyro, plus random pushes. This is what lets the network trained in MuJoCo Warp control the CPU
MuJoCo plant of this repo — and, later, the real robot.

### 2.6 PPO settings and how long it ran

| Item | Value |
|---|---|
| Algorithm | PPO, `rsl_rl` implementation (`OnPolicyRunner`) |
| Steps collected per iteration | 24 per environment × 2048 environments = **49 152 samples** |
| Learning **epochs** per iteration | 5 passes over those samples, in 4 mini-batches |
| **Iterations** | **2000** → 98.3 million simulated control steps ≈ 23 days of robot time |
| Learning rate | 1e-3, adaptive on target KL 0.01 |
| Discount γ / GAE λ | 0.99 / 0.95 |
| Clip ratio / entropy bonus | 0.2 / 0.01 |
| Observation normalization | on (running mean/std, see §4) |
| Checkpoints | `model_<iteration>.pt` every 250 iterations; the deployed one is `model_1999.pt` |
| Wall time | MLP ≈ 1.6 s/iteration ≈ **55 min**; Cartan ≈ 2.2 s/iteration ≈ **73 min** |
| Final mean reward / episode length | MLP 105 / 913 steps; Cartan 99 / 920 steps (of 1000) |

Note the two words that are often confused: an **iteration** is one cycle of "collect 49 152 samples, then
learn from them"; an **epoch** is one pass over those samples inside the learning phase (5 per iteration).
There are 2000 iterations and therefore 10 000 epochs, but "epoch" is the less useful number here.

### 2.7 What is inside `model_1999.pt`

A PyTorch checkpoint: the **actor** (the policy that runs on the robot), the **critic** (a second network that
estimates future reward — used only during training), the optimizer state, and the **observation normalizer**
(61 means and 61 standard deviations accumulated during training). Only the actor and the normalizer are
deployed.

---

## 3. The two networks: MLP and Cartan

Both were trained on the same task, same reward, same 2048 × 2000 budget, same seeds. Only the network class
inside actor and critic changes.

### 3.1 MLP (`MDK_POLICY 0`) — the reference

A plain multilayer perceptron: `61 → 512 → 256 → 128 → 14`, activation **ELU** between layers.
197 896 parameters (773 KB as float32). This is what the official Pollen policies use.

### 3.2 Cartan (`MDK_POLICY 1`) — the experimental one

A **Cartan Network** ([arXiv:2505.24353](https://arxiv.org/abs/2505.24353)) with the **DiLU** activation.
Instead of stacking linear layers, each layer works on a point of a solvable Lie group (a hyperbolic space in
"Cartan coordinates": one scalar `c`, plus a 192-vector called the *fiber* or *paint*). A layer does: a linear
map on the fiber, a group translation (`beta`), and a *fiber rotation* by a unit vector (`theta`) — a
non-linear operation involving `exp`, `log` and a dot product — then DiLU `(ELU(x) + 0.1x)/1.1` on the fiber.
The readout multiplies the fiber by `exp(c)` and applies a final linear layer.

Shape: `61 → 192 (embedding) → 3 × CartanLinear(193) → 14`. 127 054 parameters (496 KB float32, **−36 %**
vs MLP) and fewer multiply-accumulates per step. In the lab it was competitive on geometric tasks and on
quadrupeds; on the MicroDuck at 2000 iterations the MLP still has the better reward (105 vs 99) and better
speed tracking, while Cartan is lighter and faster to evaluate (0.25 ms vs 0.37 ms per step in this SITL).
Both stand and walk in this repo.

---

## 4. From `.pt` to C code in the firmware

1. **Export to ONNX** (in `microduck_rl`, `scripts/export.py`): traces `actor(normalizer(obs))`. The
   normalizer is *baked* into the graph: the first two ONNX nodes are `Sub(mean)` and `Div(std)`. This is why
   the firmware never has to compute means or standard deviations — they are constants learned during
   training, exactly like the weights.
2. **ONNX → C header** (`tools/export_policy_c.py` for the MLP, `tools/export_cartan_c.py` for Cartan):
   reads the initializers and writes `policy_mlp.h` / `policy_cartan.h` with `mean[61]`, `std[61]` and all
   weight matrices as `static const float` arrays, plus `q0`.
3. **Forward pass in C** (`tools/microduck_infer.c`, copied verbatim into `libraries/AP_MicroDuck/`):
   `microduck_forward()` for the MLP and `microduck_cartan_forward()` for Cartan. Float32, no heap, only `expf`
   and `logf` from libm.
4. **Parity tests**: `tools/parity_check.py` runs 2201 observations through ONNX Runtime and through the C
   binary — max difference 3.8e-6 (MLP), 6.0e-5 (Cartan). `tools/log_parity.py` does the same with the
   observations the *firmware* logged during a flight, against the actions the firmware actually sent
   (max 2.5e-7). The network in the firmware is the network the lab trained.

---

## 5. Architecture of the integration

Three processes in simulation; on hardware the third one is the robot.

| Process | Role | Talks to |
|---|---|---|
| **MAVProxy** (or any GCS, or `scripts/hil_test.py`) | operator: arm/disarm, sticks (`rc N pwm`), modes, parameters, telemetry | ArduRover over MAVLink, TCP 5760 |
| **ArduRover SITL** (`ardupilot/build/sitl/bin/ardurover --model JSON`) | the firmware: sensors, RC, modes, arming, **AP_MicroDuck** task, servo outputs | MAVProxy (MAVLink); plant (SIM_JSON UDP 9002 out / 9003 in) |
| **MuJoCo plant** (`plant/mujoco_json_plant.py`) | the robot body: 14 BAM-XL330 actuators, physics, IMU, joint encoders | ArduRover (SIM_JSON) |

### 5.1 The 50 Hz loop inside ArduRover

1. `AP_InertialSensor` gives gyro and accelerometer in ArduPilot's body frame (FRD: x forward, y right, z down).
2. `AP_MicroDuck::update_attitude()` (at loop rate) maintains the **gravity direction** with a small IMU-only
   complementary filter — gyro propagation, slow accelerometer correction (`MDK_ATT_TAU`). The EKF is *not*
   used for the observation, because training used the raw IMU.
3. Joint positions and velocities arrive from the plant (in SITL) or will arrive from the Dynamixel bus (on hardware).
4. Sticks → twist in SI units: `vx = stick2 × MDK_VX_MAX`, `vy = stick1 × MDK_VY_MAX`, `ωz = stick4 × MDK_WZ_MAX`.
   Mode HOLD forces the twist to zero; disarmed forces the action to zero.
5. The 61-vector is assembled in the **trunk FLU frame** (gyro `(x, −y, −z)`, gravity `(x, −y, −z)`), the C
   forward pass runs (≈0.3 ms), the 14 actions are clipped and stored as "previous action" for the next tick.
6. `q_target = q0 + action` → PWM `1500 + q_target/0.003` (1 µs = 3 mrad) → `SRV_Channels` functions
   Scripting1..14 → in SITL the JSON servo packet, on hardware the servo bus.

### 5.2 The plant protocol (SIM_JSON)

ArduPilot's SITL has a standard "JSON" physics backend: it sends a binary packet with 16 PWM values at
`SIM_RATE_HZ` and waits for one JSON line with `timestamp, imu{gyro, accel_body}, position, velocity,
quaternion` (lock-step: the simulation cannot run ahead of the firmware). This repo adds one optional field,
`joints{jpos[14], jvel[14]}`, parsed by `SIM_JSON.cpp` into `sitl->state`, which is the SITL implementation of
the joint-feedback source. `SIM_RATE_HZ 200` and MuJoCo `timestep 0.005` mean one physics step per frame,
and the 50 Hz task sees 4 sub-steps, as in training.

Frames: MuJoCo is z-up / trunk FLU, ArduPilot is NED / FRD. The plant converts with `(x, y, z) → (x, −y, −z)`
for vectors and `(w, x, y, z) → (w, x, −y, −z)` for the quaternion.

---

## 6. What was changed in ArduPilot

Fork [`virtualrobotix/ardupilot`](https://github.com/virtualrobotix/ardupilot/tree/microduck-ppo), branch
`microduck-ppo`, on top of upstream `master` (24 Sep 2026). Two commits, 7 files touched + 1 new library.

| File | Change |
|---|---|
| `libraries/AP_MicroDuck/` (new) | `AP_MicroDuck.{h,cpp}`: observation, gravity filter, action history, sticks, servo output, parameters `MDK_*`, dataflash `MDK/MDKQ/MDKV/MDKA`, `NAMED_VALUE_FLOAT PPO_*`; `microduck_infer.{h,c}`; `policy_mlp.h`, `policy_cartan.h` |
| `libraries/SITL/SIM_JSON.{h,cpp}` | parse `joints/jpos`, `joints/jvel` (type `DATA_FLOAT_ARRAY14`) |
| `libraries/SITL/SITL.h` | `sitl_fdm.joint_pos/joint_vel/joint_count/joint_time_us` |
| `Rover/Parameters.{h,cpp}` | `g2.microduck`, group `MDK_` (index 63) |
| `Rover/Rover.cpp` | scheduler: `update_attitude` @ loop rate, `update` @ 50 Hz |
| `Rover/wscript` | link `AP_MicroDuck` |

Parameters (`sitl/microduck.parm` sets them for SITL): `MDK_ENABLE`, `MDK_POLICY` (0 MLP / 1 Cartan),
`MDK_VX_MAX 0.4`, `MDK_VY_MAX 0.3`, `MDK_WZ_MAX 1.0`, `MDK_WD_MS 40` (joint-feedback watchdog),
`MDK_ATT_SRC` (0 IMU filter / 1 AHRS, debug), `MDK_ATT_TAU 0.5`, `MDK_RC_VX/VY/WZ 2/1/4`, `MDK_ACT_MAX 2.0`,
`MDK_LOG`, `MDK_HOLD_MODE 4`, `MDK_SRV_FN0 94`. Plus `SERVO1..14_FUNCTION 94..107`, `SIM_RATE_HZ 200`,
`SCHED_LOOP_RATE 200`, and **`INS_GYRO_FILTER 0`** (see §9).

Telemetry: `PPO_MS` forward time, `PPO_PGZ` gravity z (standing ≈ −1), `PPO_VX` command, `PPO_FAIL`
(0 ok, 1 disarmed, 2 no joint feedback, 3 stale feedback, 4 forward error).

---

## 7. Setup

macOS or Linux. Requires `uv` (or any Python 3.11/3.12), `ffmpeg` (video), `mavproxy.py`, a C toolchain.

```bash
git clone --recurse-submodules https://github.com/virtualrobotix/microduck-ap-ppo-sitl.git
cd microduck-ap-ppo-sitl
uv venv --python 3.12 && uv pip install -r requirements.txt
cd ardupilot && git submodule update --init --recursive --depth 1
../.venv/bin/python ./waf configure --board sitl && ../.venv/bin/python ./waf rover && cd ..
```

The plant loads the MicroDuck MJCF scene from the NOESIS EXPERIMENT checkout (`plant/mujoco_json_plant.py`,
`NOESIS_MJCF`); point `MICRODUCK_MJCF` at your copy of `microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml`.

Regenerating the C headers from other checkpoints:

```bash
.venv/bin/python tools/export_policy_c.py  my_mlp.onnx    --out ardupilot/libraries/AP_MicroDuck/policy_mlp.h    --name mlp
.venv/bin/python tools/export_cartan_c.py  my_cartan.onnx --out ardupilot/libraries/AP_MicroDuck/policy_cartan.h --name cartan
.venv/bin/python tools/parity_check.py my_mlp.onnx --name mlp            # must print PARITY OK
```

---

## 8. Running the demo and the tests

One-command visual demo (MuJoCo window + SITL + MAVProxy typed by the script):

```bash
.venv/bin/python scripts/demo_mavproxy.py                  # MLP
.venv/bin/python scripts/demo_mavproxy.py --policy 1       # Cartan
.venv/bin/python scripts/demo_mavproxy.py --video out.mp4 --keep
```

Sequence typed into MAVProxy: `param set MDK_POLICY`, `mode manual`, `rc all 1500`, `arm throttle` → 6 s
standing → `rc 2 2000` forward 3 s → `rc 2 1000` backward 2 s → `rc 1 1800` lateral 2 s → `rc 4 2000` until the
yaw rate streamed back in `ATTITUDE` integrates to 90° → `rc 2 2000` forward 5 s → `mode hold` → `mode manual`
→ `disarm`. The video overlay shows each command with the MAVLink message it produces. `--keep` leaves
everything running for manual commands.

Manual: terminal 1 `.venv/bin/mjpython plant/mujoco_json_plant.py` (macOS; `scripts/run_plant.sh` on Linux),
terminal 2 `scripts/run_sitl.sh --console`, then the same MAVProxy commands by hand.

Automatic HIL battery (SITL started headless, e.g. `scripts/run_sitl.sh --no-mavproxy`):

```bash
.venv/bin/python scripts/hil_test.py battery               # MLP
.venv/bin/python scripts/hil_test.py battery --policy 1    # Cartan
```

Steps and pass criteria: arm; stand 15 s (`PPO_FAIL 0`, `PPO_PGZ < −0.9`); forward `rc 2 1750` 12 s
(`PPO_VX ≈ 0.2`, upright); lateral; turn; HOLD (`PPO_VX = 0`); disarm (`PPO_FAIL 1`); in-situ parity of the
logged observations against ONNX.

Reference rollout without ArduPilot (policy ↔ plant only), useful to separate plant issues from firmware issues:

```bash
.venv/bin/python tools/policy_rollout_plant.py policies/microduck_mlp_2048x2000_it1999.onnx --vx 0.3 --seconds 10
```

---

## 9. Results

| | MLP (`MDK_POLICY 0`) | Cartan (`MDK_POLICY 1`) |
|---|---|---|
| Parity C vs ONNX (offline, 2201 observations) | max 3.8e-6 | max 6.0e-5 |
| Parity in-situ (firmware-logged obs → ONNX vs firmware actions) | max 2.5e-7 | p99 1.3e-6 |
| Forward pass in SITL (Apple Silicon, `-O2`) | ~0.37 ms | ~0.25 ms |
| Weights in flash (float32) | 773 KB | 496 KB |
| HIL battery (arm, stand, forward, lateral, turn, HOLD, disarm, parity) | 8/8 PASS | 8/8 PASS |
| Forward at 0.3 / 0.4 m/s command, 15 s each + turn | no fall, ~0.15–0.25 m/s achieved | — |

**The lesson that cost a morning.** The first HIL run fell after one second although observations and
network were perfect (parity exact). Cause: ArduRover's default `INS_GYRO_FILTER` is **4 Hz** (sensible for a
wheeled vehicle). The gyro reached the network tens of milliseconds late and at half amplitude. With
`INS_GYRO_FILTER 0` (or 40 Hz) the duck stands and walks. The same rule will apply on hardware: no slow
low-pass on the gyro that feeds the policy.

Speed tracking (~0.15–0.25 m/s at a 0.4 m/s command) is identical to the Python reference rollout on the same
CPU plant: it is the gap between MuJoCo-CPU and the MuJoCo-Warp training simulator, not the ArduPilot integration.

Next steps: a Dynamixel joint-feedback backend for the real robot; the forward pass benchmarked on an
STM32H7; twist commands from MAVLink GUIDED in addition to sticks.

---

## 10. Repository layout

| Path | What |
|---|---|
| `ardupilot/` | submodule: fork, branch `microduck-ppo` |
| `ardupilot/libraries/AP_MicroDuck/` | the task, the C forward pass, the generated weight headers |
| `plant/mujoco_json_plant.py` | MuJoCo plant: SIM_JSON protocol, BAM actuators, frames, optional mp4 recording with command overlay |
| `policies/` | the two validated ONNX exports (iteration 1999) |
| `tools/export_policy_c.py`, `tools/export_cartan_c.py` | ONNX → C headers |
| `tools/microduck_infer.{h,c}` | C forward pass (MLP and Cartan) |
| `tools/parity_check.py`, `tools/parity_check*.c` | ONNX Runtime vs C |
| `tools/log_parity.py` | firmware logs vs ONNX |
| `tools/policy_rollout_plant.py` | policy ↔ plant without ArduPilot |
| `sitl/microduck.parm` | SITL parameters |
| `scripts/run_plant.sh`, `scripts/run_sitl.sh` | launchers |
| `scripts/hil_test.py` | automatic battery |
| `scripts/demo_mavproxy.py` | visual demo driven through MAVProxy |
| `docs/` | project document (IT), demo video |

---

## Glossary

- **Policy** — the neural network that maps observation → action; the "controller".
- **Actor / critic** — in PPO two networks are trained: the actor is the policy; the critic estimates how much
  reward will follow from a state and is used only to train the actor.
- **Observation** — the vector of numbers the policy receives each step (here 61 floats).
- **Action** — the vector the policy outputs (here 14 joint offsets in radians).
- **Reward** — the scalar score computed by the simulator each step; training maximizes its sum.
- **Episode** — one simulated attempt, here up to 20 s; ends early on a fall.
- **Iteration** — one PPO cycle: collect samples from all parallel environments, then learn from them.
- **Epoch** — one pass over the collected samples during the learning phase (5 per iteration here).
- **Domain randomization** — varying physical parameters every episode so the policy does not overfit one simulator.
- **Observation normalization** — subtracting a running mean and dividing by a running std of every observation
  component; the statistics are frozen at export and baked into the network.
- **ONNX** — an exchange format for neural networks; here the intermediate step between PyTorch and C.
- **`.pt`** — a PyTorch checkpoint (weights + optimizer + normalizer).
- **PPO** — Proximal Policy Optimization, the RL algorithm used (`rsl_rl` implementation).
- **MLP** — multilayer perceptron, the standard fully-connected network.
- **Cartan Network / DiLU** — the alternative network family from arXiv:2505.24353, operating on a solvable
  Lie group; DiLU is its activation.
- **SITL** — Software In The Loop: the real ArduPilot firmware compiled for the PC, with simulated sensors.
- **HIL** — Hardware In The Loop; here used loosely for "firmware in the loop with an external physics plant".
- **Plant** — control-engineering term for the physical system being controlled (the robot body).
- **SIM_JSON** — ArduPilot's generic JSON physics interface for external simulators.
- **MAVLink / MAVProxy** — the telemetry protocol and the command-line ground station of ArduPilot.
- **FRD / FLU / NED** — axis conventions: Forward-Right-Down (ArduPilot body), Forward-Left-Up (MuJoCo trunk),
  North-East-Down (ArduPilot world).
- **BAM** — Better Actuator Models: the measured voltage/friction model of the XL330 servo used in training and in the plant.
- **Lock-step** — the simulator advances only when the firmware has consumed the previous frame, so time is consistent.
