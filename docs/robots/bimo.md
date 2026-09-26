# Bimo

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `bimo` |
| `NNM_ROBOT` | **3** |
| Classe | bipede |
| Progetto | Mekion |
| Stato | manca una scena MuJoCo pronta per il training |
| Giunti comandati | 8 |
| Osservazione | 33 valori |
| Frequenza policy | 25 Hz |
| Attuatori | 8× Feetech STS3215 12 V (bus) |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | Apache-2.0 |

## Repo originale e file di base

- Repository: [github.com/mekion/the-bimo-project](https://github.com/mekion/the-bimo-project)
- CAD e parti stampabili: non ancora pubblicato (upstream: coming soon)
- BOM e montaggio: non ancora pubblicata; per il setup MCU/README.md e BimoAPI/README.md
- Training upstream: Isaac Lab + rsl_rl PPO, distillata in uno studente [64, 32] su RP2040
- Policy upstream: nessuna

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot bimo
```

Nota sul modello: solo USD Isaac Lab (IsaacLab/bimo/assets/Bimo.usd); convertire USD -> MJCF e salvarlo come robots/bimo/robot/scene.xml.

## Topologia

File: [`robots/bimo/robot/profile.json`](../../robots/bimo/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `RHip` | -0.5236 | `SERVO1_FUNCTION 94` |
| 1 | `LHip` | -0.5236 | `SERVO2_FUNCTION 95` |
| 2 | `RShoulder` | +0.0000 | `SERVO3_FUNCTION 96` |
| 3 | `LShoulder` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `RKnee` | +1.0472 | `SERVO5_FUNCTION 98` |
| 5 | `LKnee` | +1.0472 | `SERVO6_FUNCTION 99` |
| 6 | `RAnkle` | +0.5236 | `SERVO7_FUNCTION 100` |
| 7 | `LAnkle` | +0.5236 | `SERVO8_FUNCTION 101` |

Osservazione (33): gyro FLU 3, gravità FLU 3, q−q0 8, q̇ 8, azione precedente 8, twist vx vy ωz 3.
Azione (8): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/bimo/robot/ppo.yaml`](../../robots/bimo/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **33 → 512 → 256 → 128 → 8**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/bimo/policies/`](../../robots/bimo/policies/) → sulla microSD `/APM/nnm/bimo/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot bimo                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot bimo --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot bimo --eval robots/bimo/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot bimo          # robot.bin dal profilo
# microSD: /APM/nnm/bimo/robot.bin  e  /APM/nnm/bimo/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 3` (riavvio), `NNM_POLICY 0`, `SERVO1..8_FUNCTION 94..101`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- L'upstream gira a 20 Hz, che non divide i 50 Hz del task AP_NNMixer: si addestra a 25 Hz.
- Le azioni upstream sono incrementi del comando giunto; il contratto NNMixer usa offset da q0, quindi la policy va riaddestrata.
