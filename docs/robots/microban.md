# Microban

| Campo | Valore |
|---|---|
| Id filesystem | `microban` |
| Classe | biped |
| Stato | `pipeline-ready` |
| DOF | see upstream MJCF (fill after scene load) |
| Rate policy | 50 Hz |
| Attuatori | BAM |
| Simulazione | MjLab / MuJoCo Warp |
| Upstream | [https://github.com/Rhoban/mjlab_microban](https://github.com/Rhoban/mjlab_microban) |
| Modello 3D / CAD | [https://github.com/Rhoban/mjlab_microban](https://github.com/Rhoban/mjlab_microban) |
| Parti stampabili | [https://github.com/Rhoban/mjlab_microban](https://github.com/Rhoban/mjlab_microban) |

## Architettura

Already MjLab with velocity task and ONNX export. First candidate after MicroDuck. BAM actuators.

Profilo macchina: [`robots/microban/robot/profile.json`](../../robots/microban/robot/profile.json).

- `n_joints`: **da riempire dopo il load della scena**
- `obs_dim`: **derivato dal profilo (gyro+gravity+3×n_joints+twist[+head/body])**
- Policy ONNX / `.nnm` solo in `robots/microban/policies/` e su SD in `/APM/nnm/microban/policies/`.

## Simulare

```bash
python tools/robots/zero_policy_step.py --robot microban
```

Addestramento (job GPU separato, stesso schema MjLab velocità):

```bash
python tools/robots/train_velocity.py --robot microban   # stub / entrypoint
python tools/robots/export_nnm.py robots/microban/policies/<run>.onnx --robot microban
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

