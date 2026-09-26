# Legolas

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

| | |
|---|---|
| Id (cartella) | `legolas` |
| `NNM_ROBOT` | **4** |
| Classe | bipede |
| Progetto | daviddoo02 |
| Stato | manca una scena MuJoCo pronta per il training |
| Giunti comandati | 10 |
| Osservazione | 39 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 10 servo hobby PWM (8× 40 kg, 2× 80 kg), upstream tramite PCA9685 |
| Collegamento | servo PWM: collegabili alle uscite dell'autopilota |
| Licenza upstream | MIT |

## Repo originale e file di base

- Repository: [github.com/daviddoo02/Legolas-an-open-source-biped](https://github.com/daviddoo02/Legolas-an-open-source-biped)
- Scena MuJoCo upstream: [`Mujoco xml/Working Mk 5 - Old model - Demo only/CMU Mk 5.xml`](https://github.com/daviddoo02/Legolas-an-open-source-biped/blob/main/Mujoco%20xml/Working%20Mk%205%20-%20Old%20model%20-%20Demo%20only/CMU%20Mk%205.xml)
- CAD e parti stampabili: [github.com/daviddoo02/Legolas-an-open-source-biped/tree/main/CAD](https://github.com/daviddoo02/Legolas-an-open-source-biped/tree/main/CAD) (SolidWorks V1-V3, STL in CAD/Legolas/V2/STLs)
- BOM e montaggio: lista materiali nel README upstream; guida di montaggio non ancora pubblicata
- Training upstream: nessuno upstream (controllore IK del passo su ROS)
- Policy upstream: nessuna

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot legolas
```

Nota sul modello: MJCF dimostrativo: giunto libero commentato, gravità 0, ctrlrange dei motori ±1e-5, gambe a catena chiusa (4 vincoli connect); va sistemato prima del training e salvato come robots/legolas/robot/scene.xml.

## Topologia

File: [`robots/legolas/robot/profile.json`](../../robots/legolas/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `R_Hip_1` | +0.0000 | `SERVO1_FUNCTION 94` |
| 1 | `R_Hip_2` | +0.0000 | `SERVO2_FUNCTION 95` |
| 2 | `R_Thigh` | +0.0000 | `SERVO3_FUNCTION 96` |
| 3 | `R_Foreleg` | +0.0000 | `SERVO4_FUNCTION 97` |
| 4 | `R_Servo` | +0.0000 | `SERVO5_FUNCTION 98` |
| 5 | `L_Hip_1` | +0.0000 | `SERVO6_FUNCTION 99` |
| 6 | `L_Hip_2` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `L_Thigh` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `L_Foreleg` | +0.0000 | `SERVO9_FUNCTION 102` |
| 9 | `L_Servo` | +0.0000 | `SERVO10_FUNCTION 103` |

Osservazione (39): gyro FLU 3, gravità FLU 3, q−q0 10, q̇ 10, azione precedente 10, twist vx vy ωz 3.
Azione (10): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/legolas/robot/ppo.yaml`](../../robots/legolas/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **39 → 512 → 256 → 128 → 10**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/legolas/policies/`](../../robots/legolas/policies/) → sulla microSD `/APM/nnm/legolas/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot legolas                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot legolas --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot legolas --eval robots/legolas/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot legolas          # robot.bin dal profilo
# microSD: /APM/nnm/legolas/robot.bin  e  /APM/nnm/legolas/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 4` (riavvio), `NNM_POLICY 0`, `SERVO1..10_FUNCTION 94..103`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- q0 non è pubblicata: va definita la posa di stand sul MJCF sistemato e scritta in catalog.py.
- I servo PWM si collegano direttamente alle uscite dell'autopilota (10 ≤ 16 funzioni servo); manca ancora in robot.bin una calibrazione per giunto (verso, centro, rad/µs).
