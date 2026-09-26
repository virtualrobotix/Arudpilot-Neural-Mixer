# Rex / SpotMicro

| Campo | Valore |
|---|---|
| Id filesystem | `rex` |
| Classe | quadruped |
| Stato | `needs-mjcf` |
| DOF | 12 (3 per leg) |
| Rate policy | 50 Hz |
| Attuatori | see SpotMicro BOM |
| Simulazione | PyBullet upstream; MuJoCo target here |
| Upstream | [https://github.com/nicrusso7/rex-gym](https://github.com/nicrusso7/rex-gym) |
| Modello 3D / CAD | [https://github.com/nicrusso7/rex-gym](https://github.com/nicrusso7/rex-gym) |
| Parti stampabili | [https://github.com/nicrusso7/rex-gym](https://github.com/nicrusso7/rex-gym) |

## Architettura

12-DoF quadruped. Upstream is PyBullet; pipeline converts URDF to MuJoCo.

Profilo macchina: [`robots/rex/robot/profile.json`](../../robots/rex/robot/profile.json).

- `n_joints`: **12**
- `obs_dim`: **derivato dal profilo (gyro+gravity+3×n_joints+twist[+head/body])**
- Policy ONNX / `.nnm` solo in `robots/rex/policies/` e su SD in `/APM/nnm/rex/policies/`.

## Simulare

```bash
python tools/robots/zero_policy_step.py --robot rex
```

Addestramento (job GPU separato, stesso schema MjLab velocità):

```bash
python tools/robots/train_velocity.py --robot rex   # stub / entrypoint
python tools/robots/export_nnm.py robots/rex/policies/<run>.onnx --robot rex
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

