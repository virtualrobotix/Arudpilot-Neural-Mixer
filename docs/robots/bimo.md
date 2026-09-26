# Bimo

| Campo | Valore |
|---|---|
| Id filesystem | `bimo` |
| Classe | biped |
| Stato | `needs-mjcf` |
| DOF | ~8 joints |
| Rate policy | 20 Hz |
| Attuatori | STS-3215 |
| Simulazione | Isaac Lab upstream; MuJoCo target here |
| Upstream | [https://github.com/mekion/the-bimo-project](https://github.com/mekion/the-bimo-project) |
| Modello 3D / CAD | [https://github.com/mekion/the-bimo-project](https://github.com/mekion/the-bimo-project) |
| Parti stampabili | [https://github.com/mekion/the-bimo-project](https://github.com/mekion/the-bimo-project) |

## Architettura

~8 STS-3215 servos, 20 Hz control. Trained in Isaac Lab upstream; last in the local train order.

Profilo macchina: [`robots/bimo/robot/profile.json`](../../robots/bimo/robot/profile.json).

- `n_joints`: **8**
- `obs_dim`: **derivato dal profilo (gyro+gravity+3×n_joints+twist[+head/body])**
- Policy ONNX / `.nnm` solo in `robots/bimo/policies/` e su SD in `/APM/nnm/bimo/policies/`.

## Simulare

```bash
python tools/robots/zero_policy_step.py --robot bimo
```

Addestramento (job GPU separato, stesso schema MjLab velocità):

```bash
python tools/robots/train_velocity.py --robot bimo   # stub / entrypoint
python tools/robots/export_nnm.py robots/bimo/policies/<run>.onnx --robot bimo
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

