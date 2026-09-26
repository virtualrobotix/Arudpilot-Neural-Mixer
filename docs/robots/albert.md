# AlbertPro

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

<img src="img/albert.jpg" alt="AlbertPro" width="360">

*AlbertPro, render MuJoCo (il repo pubblica solo render). Foto: [github.com/thinking0things/AlbertPro](https://github.com/thinking0things/AlbertPro).*

| | |
|---|---|
| Id (cartella) | `albert` |
| `NNM_ROBOT` | **8** |
| Classe | quadrupede |
| Progetto | thinking0things |
| Stato | scena MuJoCo nativa pronta; policy da addestrare |
| Giunti comandati | 8 |
| Osservazione | 33 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 8 servo PWM tramite PCA9685 (I²C) su ESP32 |
| Collegamento | servo PWM: collegabili alle uscite dell'autopilota |
| Licenza upstream | MIT |

## Repo originale e file di base

- Repository: [github.com/thinking0things/AlbertPro](https://github.com/thinking0things/AlbertPro)
- Scena MuJoCo upstream: [`RL/dog.xml`](https://github.com/thinking0things/AlbertPro/blob/main/RL/dog.xml)
- CAD e parti stampabili: [github.com/thinking0things/AlbertPro/tree/main/hardware](https://github.com/thinking0things/AlbertPro/tree/main/hardware) (mesh STL per la simulazione in RL/meshes)
- BOM e montaggio: [github.com/thinking0things/AlbertPro#robot](https://github.com/thinking0things/AlbertPro#robot) (corpo 14 × 11 × 2 cm, 8 servo, PCA9685, ESP32)
- Training upstream: PPO + GAE in MuJoCo (notebook in RL/), 100 Hz, azioni ΔΔθ su un buffer di delta
- Policy upstream: RL/models/ policy.h (MLP 24-64-8 ReLU/tanh, osservazione senza IMU, azioni ΔΔθ); non convertibile

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot albert
```

## Topologia

File: [`robots/albert/robot/profile.json`](../../robots/albert/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `FL_hip` | +0.9000 | `SERVO1_FUNCTION 94` |
| 1 | `FL_knee` | -1.4000 | `SERVO2_FUNCTION 95` |
| 2 | `FR_hip` | +0.9000 | `SERVO3_FUNCTION 96` |
| 3 | `FR_knee` | -1.4000 | `SERVO4_FUNCTION 97` |
| 4 | `RL_hip` | +0.9000 | `SERVO5_FUNCTION 98` |
| 5 | `RL_knee` | -1.4000 | `SERVO6_FUNCTION 99` |
| 6 | `RR_hip` | +0.9000 | `SERVO7_FUNCTION 100` |
| 7 | `RR_knee` | -1.4000 | `SERVO8_FUNCTION 101` |

Osservazione (33): gyro FLU 3, gravità FLU 3, q−q0 8, q̇ 8, azione precedente 8, twist vx vy ωz 3.
Azione (8): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/albert/robot/ppo.yaml`](../../robots/albert/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **33 → 512 → 256 → 128 → 8**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/albert/policies/`](../../robots/albert/policies/) → sulla microSD `/APM/nnm/albert/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot albert                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot albert --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot albert --eval robots/albert/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot albert          # robot.bin dal profilo
# microSD: /APM/nnm/albert/robot.bin  e  /APM/nnm/albert/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 8` (riavvio), `NNM_POLICY 0`, `SERVO1..8_FUNCTION 94..101`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- La scena upstream è già MuJoCo nativa (attuatori di posizione kp 60, IMU sul tronco): fetch_upstream.py la usa così com'è.
- L'upstream gira a 100 Hz; il task AP_NNMixer arriva a 50 Hz, quindi si addestra a 50 Hz.
- La policy upstream non vede l'IMU e comanda accelerazioni dei giunti (ΔΔθ): per l'autopilota va riaddestrata sul contratto NNMixer (offset da q0).
- I servo PWM si collegano direttamente alle uscite dell'autopilota (8 ≤ 16); manca ancora in robot.bin la calibrazione per giunto.
