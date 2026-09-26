# MicroDuck (Pollen / Hugging Face)

| Campo | Valore |
|---|---|
| Id filesystem | `microduck` |
| Classe | biped |
| Stato | `integrated` |
| DOF | 14 hinge joints + trunk free joint |
| Rate policy | 50 Hz |
| Attuatori | Dynamixel XL330 (BAM M6 model in sim) |
| Simulazione | MuJoCo / mjlab (official training stack) |
| Upstream | [https://github.com/pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) |
| Modello 3D / CAD | [https://huggingface.co/spaces/pollen-robotics/microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator) |
| Parti stampabili | [https://huggingface.co/spaces/pollen-robotics/microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator) |

## Architettura

Reference biped already running in AP_NNMixer. MJCF from Pollen mjlab_microduck. 14 Dynamixel XL330.

Profilo macchina: [`robots/microduck/robot/profile.json`](../../robots/microduck/robot/profile.json).

- `n_joints`: **14**
- `obs_dim`: **61**
- Policy ONNX / `.nnm` solo in `robots/microduck/policies/` e su SD in `/APM/nnm/microduck/policies/`.

## Simulare

```bash
python tools/robots/zero_policy_step.py --robot microduck
```

Addestramento (job GPU separato, stesso schema MjLab velocità):

```bash
python tools/robots/train_velocity.py --robot microduck   # stub / entrypoint
python tools/robots/export_nnm.py robots/microduck/policies/<run>.onnx --robot microduck
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

