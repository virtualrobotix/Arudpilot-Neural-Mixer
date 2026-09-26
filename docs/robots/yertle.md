# Yertle

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `yertle` |
| `NNM_ROBOT` | **7** |
| Classe | quadrupede |
| Progetto | Jerome Graves |
| Stato | scena MuJoCo generata dall'URDF; policy da addestrare |
| Giunti comandati | 12 |
| Osservazione | 45 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 12× SPT5435LV-180W 35 kg PWM, upstream tramite PCA9685 su ESP32 |
| Collegamento | servo PWM: collegabili alle uscite dell'autopilota |
| Licenza upstream | MIT |

## Repo originale e file di base

- Repository: [github.com/Jerome-Graves/yertle](https://github.com/Jerome-Graves/yertle)
- URDF upstream: [`simulation/yertle.urdf`](https://github.com/Jerome-Graves/yertle/blob/main/simulation/yertle.urdf)
- CAD e parti stampabili: [github.com/Jerome-Graves/yertle/tree/main/design](https://github.com/Jerome-Graves/yertle/tree/main/design) (CAD/Yertle_Single_v2.step, STL/)
- BOM e montaggio: [github.com/Jerome-Graves/yertle/blob/main/design/README.md](https://github.com/Jerome-Graves/yertle/blob/main/design/README.md)
- Training upstream: SB3 PPO su PyBullet; Isaac Lab + rsl_rl
- Policy upstream: nessuna

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot yertle
```

Lo script compila l'URDF con MuJoCo e scrive `robots/yertle/robot/scene.xml` con giunto libero, pavimento, IMU sul tronco e attuatori di posizione.

## Topologia

File: [`robots/yertle/robot/profile.json`](../../robots/yertle/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `lf_shoulder` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `lf_thigh` | -0.6000 | `SERVO2_FUNCTION 95` |
| 2 | `lf_shin` | +0.9000 | `SERVO3_FUNCTION 96` |
| 3 | `rf_shoulder` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `rf_thigh` | -0.6000 | `SERVO5_FUNCTION 98` |
| 5 | `rf_shin` | +0.9000 | `SERVO6_FUNCTION 99` |
| 6 | `lb_shoulder` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `lb_thigh` | -0.6000 | `SERVO8_FUNCTION 101` |
| 8 | `lb_shin` | +0.9000 | `SERVO9_FUNCTION 102` |
| 9 | `rb_shoulder` | +0.0000 | `SERVO10_FUNCTION 103` |
| 10 | `rb_thigh` | -0.6000 | `SERVO11_FUNCTION 104` |
| 11 | `rb_shin` | +0.9000 | `SERVO12_FUNCTION 105` |

Osservazione (45): gyro FLU 3, gravità FLU 3, q−q0 12, q̇ 12, azione precedente 12, twist vx vy ωz 3.
Azione (12): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/yertle/robot/ppo.yaml`](../../robots/yertle/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **45 → 512 → 256 → 128 → 12**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/yertle/policies/`](../../robots/yertle/policies/) → sulla microSD `/APM/nnm/yertle/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot yertle                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot yertle --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot yertle --eval robots/yertle/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot yertle          # robot.bin dal profilo
# microSD: /APM/nnm/yertle/robot.bin  e  /APM/nnm/yertle/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 7` (riavvio), `NNM_POLICY 0`, `SERVO1..12_FUNCTION 94..105`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- L'osservazione upstream (48) è il formato NNMixer più la velocità lineare della base, che l'autopilota non misura; il contratto NNMixer la toglie (45).
- I servo PWM si collegano direttamente alle uscite dell'autopilota (12 ≤ 16); manca ancora in robot.bin la calibrazione per giunto.
