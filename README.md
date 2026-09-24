# MicroDuck su ArduPilot — PPO come task del firmware, MuJoCo come pianta SITL

La policy PPO di locomozione del MicroDuck (61 osservazioni → 14 offset di posizione dei servo, 50 Hz)
gira **dentro ArduRover** come task dello scheduler (`libraries/AP_MicroDuck`). MuJoCo non è un secondo
autopilota: è il **backend fisico del SITL** (protocollo `SIM_JSON`), come Gazebo per un drone.
Il task legge solo API ArduPilot (`AP_InertialSensor`, joint feedback, RC), quindi lo stesso sorgente
è pronto per il microcontrollore: la rete è C puro con il normalizer del training incluso.

Documento di progetto: [`docs/progetto-microduck-ardupilot-ppo.md`](docs/progetto-microduck-ardupilot-ppo.md).
I pesi (ONNX) vengono dal repo NOESIS EXPERIMENT (`artifacts/microduck_cartan/checkpoints`).

```
plant/mujoco_json_plant.py  ──UDP 9002 (PWM 14 ch)──▶  ardurover SITL --model JSON
        ▲                                                  │  libraries/SITL/SIM_JSON  (+ joints jpos/jvel)
        └──── JSON: imu, quaternion, position, joints ─────┘  libraries/AP_MicroDuck   (obs → C forward → SRV 94..107)
                                                               MAVProxy / scripts/hil_test.py  (stick, arm, telemetria PPO_*)
```

## Layout

| Path | Cosa |
|---|---|
| `ardupilot/` | submodule: fork `virtualrobotix/ardupilot`, branch `microduck-ppo` (master upstream + `AP_MicroDuck`, `SIM_JSON` joints, task Rover) |
| `ardupilot/libraries/AP_MicroDuck/` | task 50 Hz: obs (IMU FRD→FLU, gravity filter IMU-only, joint feedback, stick→twist SI, storia), forward C, parametri `MDK_*`, log `MDK*` |
| `ardupilot/libraries/AP_MicroDuck/policy_mlp.h` | pesi MLP + `mean/std` generati da `tools/export_policy_c.py` (773 KB float32) |
| `plant/mujoco_json_plant.py` | pianta MuJoCo (scena e attuatori BAM XL330 del training), protocollo JSON, frame FRD/NED |
| `policies/` | ONNX validati: MLP `mlp_2048x2000` it1999, Cartan `cartan_ac_2048x2000` it1999 |
| `tools/export_policy_c.py` | ONNX → header C |
| `tools/microduck_infer.{h,c}` | forward C (identico a quello nel firmware) |
| `tools/parity_check.py` | parità ONNX Runtime vs C (max err 4e-6) |
| `tools/policy_rollout_plant.py` | rollout di riferimento: ONNX ↔ pianta senza ArduPilot |
| `tools/log_parity.py` | rigioca le obs loggate dal firmware (`MDK*`) nell’ONNX e confronta con le azioni del firmware |
| `sitl/microduck.parm` | parametri SITL: servo 1–14 = Scripting1..14, `SIM_RATE_HZ 200`, `INS_GYRO_FILTER 0`, `MDK_ENABLE 1` |
| `scripts/run_plant.sh`, `scripts/run_sitl.sh` | avvio pianta e SITL (MAVProxy console) |
| `scripts/hil_test.py` | batteria HIL automatica via MAVLink (arm, stand, avanti, laterale, rotazione, HOLD, disarm) |

## Setup

```bash
uv venv --python 3.12 && uv pip install -r requirements.txt
git submodule update --init            # ardupilot (fork, branch microduck-ppo)
cd ardupilot && git submodule update --init --recursive --depth 1
../.venv/bin/python ./waf configure --board sitl && ../.venv/bin/python ./waf rover
```

La pianta usa la scena MuJoCo del repo NOESIS (`MICRODUCK_MJCF` per cambiarla).

## Uso

Terminale 1 — pianta (viewer MuJoCo):

```bash
scripts/run_plant.sh
```

Terminale 2 — SITL + MAVProxy:

```bash
scripts/run_sitl.sh --console
```

In MAVProxy: `arm throttle` (il duck parte in piedi), `rc 2 1750` avanti ~0.2 m/s, `rc 1 1650` laterale,
`rc 4 1750` rotazione ~0.5 rad/s, `rc 2 1500` stop, `mode HOLD` forza twist 0, `disarm`.
Telemetria: `PPO_MS` (tempo forward), `PPO_PGZ` (gravità proiettata z, in piedi ≈ −1), `PPO_VX`, `PPO_FAIL`
(0 ok, 1 disarmato, 2 nessun feedback giunti, 3 feedback vecchio, 4 errore forward).

Batteria automatica (SITL avviato con `scripts/run_sitl.sh --no-mavproxy`, oppure il binario diretto):

```bash
.venv/bin/python scripts/hil_test.py battery
```

## Contratto rete

`obs[61]` float32 SI nel frame trunk FLU: gyro(3) rad/s, gravità proiettata(3), `q−q0`(14) rad, `q̇`(14) rad/s,
azione precedente(14), twist `vx,vy` m/s `ωz` rad/s, testa(4)=0, corpo(6)=0. `act[14]` rad, `q_target = q0 + act`.
Filo servo: `pwm = 1500 + q_target / 0.003` (1 µs = 3 mrad), `SERVOn_MIN 800 / MAX 2200`.

## Risultati (24/09/2026, MLP)

- Parità C vs ONNX: max 3.8e-6; parità in-situ (obs loggate dal firmware → ONNX vs azioni firmware): p99 1.2e-6.
- Forward MLP nel SITL: ~0.37 ms (host Apple Silicon).
- Batteria HIL: arm, stand 15 s, avanti 0.2/0.3/0.4 m/s, laterale, rotazione, HOLD, disarm — tutti PASS, nessuna caduta.
- Lezione sim2sim: Rover ha `INS_GYRO_FILTER` 4 Hz di default; con quel filtro il bipede cade in ~1 s
  (gyro in ritardo di decine di ms). Con filtro 0/40 Hz la policy è stabile. La pianta vale come il rollout
  Python di riferimento; il tracking di velocità è quello del sim2sim CPU (~0.15 m/s a comando 0.4).

Prossimi passi: Cartan in C (`MDK_POLICY 1`), backend Dynamixel per il joint feedback su hardware,
benchmark del forward su STM32H7.
