# MicroDuck

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `microduck` |
| `NNM_ROBOT` | **0** |
| Classe | bipede |
| Progetto | Pollen Robotics / Hugging Face |
| Stato | policy int8 disponibile; la stessa rete in float32 è validata in SITL e HIL |
| Giunti comandati | 14 |
| Osservazione | 61 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 14× Dynamixel XL330 (bus) |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | vedi upstream |

## Repo originale e file di base

- Repository: [github.com/pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl)
- Scena MuJoCo upstream: [`src/mjlab_microduck/robot/microduck/scene.xml`](https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/robot/microduck/scene.xml)
- CAD e parti stampabili: [huggingface.co/spaces/pollen-robotics/microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
- BOM e montaggio: [huggingface.co/spaces/pollen-robotics/microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
- Training upstream: mjlab 1.3.0 + rsl_rl PPO (MuJoCo Warp)
- Policy upstream: addestrata nel laboratorio NOESIS EXPERIMENT; ONNX in policies/ di questo repo

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot microduck
```

## Topologia

File: [`robots/microduck/robot/profile.json`](../../robots/microduck/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `left_hip_yaw` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `left_hip_roll` | -0.0873 | `SERVO2_FUNCTION 95` |
| 2 | `left_hip_pitch` | -0.4579 | `SERVO3_FUNCTION 96` |
| 3 | `left_knee` | -0.0049 | `SERVO4_FUNCTION 97` |
| 4 | `left_ankle` | +0.4530 | `SERVO5_FUNCTION 98` |
| 5 | `neck_pitch` | +0.3491 | `SERVO6_FUNCTION 99` |
| 6 | `head_pitch` | +0.3491 | `SERVO7_FUNCTION 100` |
| 7 | `head_yaw` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `head_roll` | +0.0000 | `SERVO9_FUNCTION 102` |
| 9 | `right_hip_yaw` | +0.0000 | `SERVO10_FUNCTION 103` |
| 10 | `right_hip_roll` | +0.0873 | `SERVO11_FUNCTION 104` |
| 11 | `right_hip_pitch` | +0.4579 | `SERVO12_FUNCTION 105` |
| 12 | `right_knee` | +0.0049 | `SERVO13_FUNCTION 106` |
| 13 | `right_ankle` | -0.4530 | `SERVO14_FUNCTION 107` |

Osservazione (61): gyro FLU 3, gravità FLU 3, q−q0 14, q̇ 14, azione precedente 14, twist vx vy ωz 3, comandi testa/corpo 10 (zero sull'autopilota).
Azione (14): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/microduck/robot/ppo.yaml`](../../robots/microduck/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **61 → 512 → 256 → 128 → 14**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/microduck/policies/`](../../robots/microduck/policies/) → sulla microSD `/APM/nnm/microduck/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

| File | Dimensione | Descrizione |
|---|---:|---|
| `walk.nnm` | 200 KB | MLP 61-512-256-128-14 (2048 env x 2000 iterazioni), int8 per riga ricavato dall'ONNX float32 validato. Nell'ambiente a contratto: in piedi 10 s, avanti 10 s (~0,10 m/s a comando 0,3), rotazione 10 s, nessuna caduta. |

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot microduck                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot microduck --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot microduck --eval robots/microduck/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot microduck          # robot.bin dal profilo
# microSD: /APM/nnm/microduck/robot.bin  e  /APM/nnm/microduck/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 0` (riavvio), `NNM_POLICY 0`, `SERVO1..14_FUNCTION 94..107`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Integrazione di riferimento: SITL, batteria HIL 8/8 e HIL hardware su Pixhawk 6C Mini con la build float32.
- Il firmware comanda 14 funzioni servo (Scripting1..14). Il robot reale richiede un backend bus Dynamixel nel firmware, non ancora scritto; SITL e HIL usano il trasporto SIM_JSON / MAVLink.
