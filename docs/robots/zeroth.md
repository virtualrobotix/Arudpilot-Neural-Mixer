# Zeroth-01

| Campo | Valore |
|---|---|
| Id filesystem | `zeroth` |
| Classe | biped |
| Stato | `needs-mjcf` |
| DOF | see upstream URDF |
| Rate policy | 50 Hz |
| Attuatori | see upstream |
| Simulazione | ksim upstream; MuJoCo target in this repo |
| Upstream | [https://github.com/zeroth-robotics/zeroth-bot](https://github.com/zeroth-robotics/zeroth-bot) |
| Modello 3D / CAD | [https://github.com/zeroth-robotics/zeroth-bot](https://github.com/zeroth-robotics/zeroth-bot) |
| Parti stampabili | [https://github.com/zeroth-robotics/zeroth-bot](https://github.com/zeroth-robotics/zeroth-bot) |

## Architettura

Printable humanoid with MJCF/URDF. Upstream trains in ksim; pipeline converts to MjLab velocity task.

Profilo macchina: [`robots/zeroth/robot/profile.json`](../../robots/zeroth/robot/profile.json).

- `n_joints`: **da riempire dopo il load della scena**
- `obs_dim`: **derivato dal profilo (gyro+gravity+3×n_joints+twist[+head/body])**
- Policy ONNX / `.nnm` solo in `robots/zeroth/policies/` e su SD in `/APM/nnm/zeroth/policies/`.

## Simulare

```bash
python tools/robots/zero_policy_step.py --robot zeroth
```

Addestramento (job GPU separato, stesso schema MjLab velocità):

```bash
python tools/robots/train_velocity.py --robot zeroth   # stub / entrypoint
python tools/robots/export_nnm.py robots/zeroth/policies/<run>.onnx --robot zeroth
```

## Costruire e collegare

Seguire la BOM e le istruzioni di montaggio nel repo upstream. Poi:

## Collegamento all'autopilota

1. Flight controller con IMU (Pixhawk 6C Mini target `Pixhawk6C-NNMixer` o SITL).
2. Feedback giunti (bus Dynamixel/STS o plant SITL) nello stesso ordine di `joint_names` del profilo.
3. Servo mappati da `NNM_SRV_FN0` in poi (default Scripting1 = 94).
4. Copiare su microSD `/APM/nnm/<id>/robot.bin` e almeno una policy in `policies/`.
5. Al boot: `NNM_ROBOT=<indice>`, `NNM_ENABLE=1`, armare in MANUAL o HOLD; twist da stick o dai modi GUIDED/AUTO/RTL.
6. Switch policy: `NNM_POLICY=<indice file>` solo da fermo / HOLD; il firmware copia da SD nello slot int8 libero e fonde le uscite ~0,5 s.

