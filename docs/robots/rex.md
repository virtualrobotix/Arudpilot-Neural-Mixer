# Rex / SpotMicro

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

<img src="img/rex.jpg" alt="Rex / SpotMicro" width="360">

*Rex / SpotMicro, SpotMicro montato. Foto: [github.com/nicrusso7/rex-gym](https://github.com/nicrusso7/rex-gym).*

| | |
|---|---|
| Id (cartella) | `rex` |
| `NNM_ROBOT` | **6** |
| Classe | quadrupede |
| Progetto | nicrusso7 (SpotMicroAI community design) |
| Stato | scena MuJoCo generata dall'URDF; policy da addestrare |
| Giunti comandati | 12 |
| Osservazione | 45 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 12× MG996R PWM |
| Collegamento | servo PWM: collegabili alle uscite dell'autopilota |
| Licenza upstream | Apache-2.0 |

## Repo originale e file di base

- Repository: [github.com/nicrusso7/rex-gym](https://github.com/nicrusso7/rex-gym)
- URDF upstream: [`rex_gym/util/pybullet_data/assets/urdf/rex.urdf`](https://github.com/nicrusso7/rex-gym/blob/master/rex_gym/util/pybullet_data/assets/urdf/rex.urdf)
- CAD e parti stampabili: [www.thingiverse.com/thing:3445283](https://www.thingiverse.com/thing:3445283) e [github.com/FlorianWilk/SpotMicroAI](https://github.com/FlorianWilk/SpotMicroAI)
- BOM e montaggio: [github.com/nicrusso7/rexctl/wiki/Mark-I](https://github.com/nicrusso7/rexctl/wiki/Mark-I)
- Training upstream: PyBullet + PPO TensorFlow 1 (ibrido open-loop + residuo)
- Policy upstream: checkpoint TensorFlow rex_gym/policies/*/model.ckpt-* (osservazione di 4 valori, azioni residue); non convertibili

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot rex
```

Lo script compila l'URDF con MuJoCo e scrive `robots/rex/robot/scene.xml` con giunto libero, pavimento, IMU sul tronco e attuatori di posizione.

## Topologia

File: [`robots/rex/robot/profile.json`](../../robots/rex/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `motor_front_left_shoulder` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `motor_front_left_leg` | -0.8864 | `SERVO2_FUNCTION 95` |
| 2 | `foot_motor_front_left` | +1.3020 | `SERVO3_FUNCTION 96` |
| 3 | `motor_front_right_shoulder` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `motor_front_right_leg` | -0.8864 | `SERVO5_FUNCTION 98` |
| 5 | `foot_motor_front_right` | +1.3020 | `SERVO6_FUNCTION 99` |
| 6 | `motor_rear_left_shoulder` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `motor_rear_left_leg` | -0.8864 | `SERVO8_FUNCTION 101` |
| 8 | `foot_motor_rear_left` | +1.3020 | `SERVO9_FUNCTION 102` |
| 9 | `motor_rear_right_shoulder` | +0.0000 | `SERVO10_FUNCTION 103` |
| 10 | `motor_rear_right_leg` | -0.8864 | `SERVO11_FUNCTION 104` |
| 11 | `foot_motor_rear_right` | +1.3020 | `SERVO12_FUNCTION 105` |

Osservazione (45): gyro FLU 3, gravità FLU 3, q−q0 12, q̇ 12, azione precedente 12, twist vx vy ωz 3.
Azione (12): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/rex/robot/ppo.yaml`](../../robots/rex/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **45 → 512 → 256 → 128 → 12**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/rex/policies/`](../../robots/rex/policies/) → sulla microSD `/APM/nnm/rex/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot rex                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot rex --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot rex --eval robots/rex/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot rex          # robot.bin dal profilo
# microSD: /APM/nnm/rex/robot.bin  e  /APM/nnm/rex/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 6` (riavvio), `NNM_POLICY 0`, `SERVO1..12_FUNCTION 94..105`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Solo URDF: fetch_upstream.py genera robots/rex/robot/scene.xml (giunto libero, pavimento, IMU, attuatori di posizione). Guadagni e limiti di coppia degli attuatori sono stime da verificare.
- I servo PWM si collegano direttamente alle uscite dell'autopilota (12 ≤ 16); manca ancora in robot.bin la calibrazione per giunto.
