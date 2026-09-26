# Open Duck Mini v2

[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · [README](../../README.it.md)

<img src="img/openduck.jpg" alt="Open Duck Mini v2" width="360">

*Open Duck Mini v2, robot montato. Foto: [github.com/apirrone/Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini).*

| | |
|---|---|
| Id (cartella) | `openduck` |
| `NNM_ROBOT` | **9** |
| Classe | bipede |
| Progetto | Antoine Pirrone (apirrone) e comunità, supporto Hugging Face / Pollen Robotics |
| Stato | scena MuJoCo nativa pronta; policy da addestrare |
| Giunti comandati | 14 |
| Osservazione | 51 valori |
| Frequenza policy | 50 Hz |
| Attuatori | 14× Feetech STS3215 (bus seriale); runtime upstream su Raspberry Pi Zero 2W |
| Collegamento | servo su bus seriale: serve il backend bus nel firmware (non ancora scritto) |
| Licenza upstream | Apache-2.0 (Open_Duck_Mini); Open_Duck_Playground senza licenza dichiarata |

## Repo originale e file di base

- Repository: [github.com/apirrone/Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini)
- Modello MuJoCo e training: [github.com/apirrone/Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground)
- Scena MuJoCo upstream: [`playground/open_duck_mini_v2/xmls/scene_flat_terrain.xml`](https://github.com/apirrone/Open_Duck_Playground/blob/main/playground/open_duck_mini_v2/xmls/scene_flat_terrain.xml)
- CAD e parti stampabili: Onshape [cad.onshape.com/documents/64074dfcfa379b37d8a47762](https://cad.onshape.com/documents/64074dfcfa379b37d8a47762) e [github.com/apirrone/Open_Duck_Mini/tree/v2/print](https://github.com/apirrone/Open_Duck_Mini/tree/v2/print) (guida di stampa)
- BOM e montaggio: [tnkr.ai/explore/docs/open-duck-mini/open-duck-mini-v2](https://tnkr.ai/explore/docs/open-duck-mini/open-duck-mini-v2) (guida di montaggio) e BOM Google Sheets linkata nel README upstream (sotto 400 $)
- Training upstream: MuJoCo Playground (JAX / Brax PPO) con reference motion per imitazione, 50 Hz; modelli attuatore identificati con BAM
- Policy upstream: BEST_WALK_ONNX.onnx e BEST_WALK_ONNX_2.onnx nella radice del repo (MLP 101-512-256-128-28, attivazione swish, uscita tanh, osservazione con fase e riferimenti di imitazione); non convertibili

Per scaricare il repo originale e preparare la scena usata da simulazione e training:

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot openduck
```

## Topologia

File: [`robots/openduck/robot/profile.json`](../../robots/openduck/robot/profile.json) → `robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.

| # | Giunto | q0 (rad) | Uscita servo |
|---:|---|---:|---|
| 0 | `left_hip_yaw` | +0.0020 | `SERVO1_FUNCTION 94` |
| 1 | `left_hip_roll` | +0.0530 | `SERVO2_FUNCTION 95` |
| 2 | `left_hip_pitch` | -0.6300 | `SERVO3_FUNCTION 96` |
| 3 | `left_knee` | +1.3680 | `SERVO4_FUNCTION 97` |
| 4 | `left_ankle` | -0.7840 | `SERVO5_FUNCTION 98` |
| 5 | `neck_pitch` | +0.0000 | `SERVO6_FUNCTION 99` |
| 6 | `head_pitch` | +0.0000 | `SERVO7_FUNCTION 100` |
| 7 | `head_yaw` | +0.0000 | `SERVO8_FUNCTION 101` |
| 8 | `head_roll` | +0.0000 | `SERVO9_FUNCTION 102` |
| 9 | `right_hip_yaw` | -0.0030 | `SERVO10_FUNCTION 103` |
| 10 | `right_hip_roll` | -0.0650 | `SERVO11_FUNCTION 104` |
| 11 | `right_hip_pitch` | +0.6350 | `SERVO12_FUNCTION 105` |
| 12 | `right_knee` | +1.3790 | `SERVO13_FUNCTION 106` |
| 13 | `right_ankle` | -0.7960 | `SERVO14_FUNCTION 107` |

Osservazione (51): gyro FLU 3, gravità FLU 3, q−q0 14, q̇ 14, azione precedente 14, twist vx vy ωz 3.
Azione (14): offset in radianti, `q_target = q0 + azione`.

## Architettura PPO

File: [`robots/openduck/robot/ppo.yaml`](../../robots/openduck/robot/ppo.yaml). Stessa rete di MicroDuck, già provata su Pixhawk 6C: **51 → 512 → 256 → 128 → 14**, ELU, normalizzazione dell'osservazione incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo (KL 0,01), γ 0,99, λ 0,95, clip 0,2.

## Policy

Cartella: [`robots/openduck/policies/`](../../robots/openduck/policies/) → sulla microSD `/APM/nnm/openduck/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.

Nessuna policy ancora: va addestrata (sezione successiva).

## Training compatibile con ArduPilot

L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del profilo. Dettagli in [training.md](training.md).

```bash
.venv/bin/python tools/robots/nnm_env.py --robot openduck                       # carica la scena, passo a policy zero
.venv/bin/python tools/robots/train_velocity.py --robot openduck --envs 8 --iters 200 --name walk
.venv/bin/python tools/robots/train_velocity.py --robot openduck --eval robots/openduck/policies/walk.nnm
```

Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.

## Deploy sull'autopilota

```bash
python tools/robots/pack_robot_bin.py --robot openduck          # robot.bin dal profilo
# microSD: /APM/nnm/openduck/robot.bin  e  /APM/nnm/openduck/policies/*.nnm
```

Parametri: `NNM_ENABLE 1`, `NNM_ROBOT 9` (riavvio), `NNM_POLICY 0`, `SERVO1..14_FUNCTION 94..107`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.

## Note

- Stessi 14 giunti di MicroDuck, nello stesso ordine: stesso contratto di osservazione e di azione, con q0 e dimensioni proprie (robot alto 42 cm).
- La scena del Playground è MuJoCo nativa (attuatori di posizione STS3215, IMU sulla base): fetch_upstream.py la usa così com'è.
- Le policy pubblicate usano swish e tanh e un'osservazione di 101 valori con fase del passo: AP_NNMixer esegue solo MLP ELU sul contratto NNMixer, quindi la policy va riaddestrata.
- Il robot reale richiede il backend bus Feetech nel firmware (14 ≤ 16 funzioni servo, quindi SITL e HIL funzionano come per MicroDuck).
