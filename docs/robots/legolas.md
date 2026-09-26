# Legolas

| Campo | Valore |
|---|---|
| Id filesystem | `legolas` |
| Classe | biped |
| Stato | `needs-mjcf` |
| DOF | ~10 joints |
| Rate policy | 50 Hz |
| Attuatori | hobby servos (see upstream) |
| Simulazione | MuJoCo XML upstream |
| Upstream | [https://github.com/daviddoo02/Legolas-an-open-source-biped](https://github.com/daviddoo02/Legolas-an-open-source-biped) |
| Modello 3D / CAD | [https://github.com/daviddoo02/Legolas-an-open-source-biped](https://github.com/daviddoo02/Legolas-an-open-source-biped) |
| Parti stampabili | [https://github.com/daviddoo02/Legolas-an-open-source-biped](https://github.com/daviddoo02/Legolas-an-open-source-biped) |

## Architettura

Cassie-like biped with MuJoCo XML in the upstream repo (~10 servos).

Profilo macchina: [`robots/legolas/robot/profile.json`](../../robots/legolas/robot/profile.json).

- `n_joints`: **10**
- `obs_dim`: **derivato dal profilo (gyro+gravity+3×n_joints+twist[+head/body])**
- Policy ONNX / `.nnm` solo in `robots/legolas/policies/` e su SD in `/APM/nnm/legolas/policies/`.

## Simulare

```bash
python tools/robots/zero_policy_step.py --robot legolas
```

Addestramento (job GPU separato, stesso schema MjLab velocità):

```bash
python tools/robots/train_velocity.py --robot legolas   # stub / entrypoint
python tools/robots/export_nnm.py robots/legolas/policies/<run>.onnx --robot legolas
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

