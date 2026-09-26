# Pipeline MuJoCo → ONNX → `.nnm` per AP_NNMixer

Ogni robot vive in `robots/<id>/`:

```
robots/<id>/
  robot/
    profile.json   # topologia: n_joints, obs_dim, rate_hz, q0, joint_names
    robot.bin      # generato: python tools/robots/pack_robot_bin.py --robot <id>
    scene.xml      # MJCF locale (MicroDuck usa NNMIXER_MJCF / scena Pollen)
    SCENE.md
  policies/
    <nome>.onnx    # dopo il training
    <nome>.nnm     # int8 per SD: python tools/robots/export_nnm.py ...
  README.md
```

Su microSD la stessa classificazione:

```
/APM/nnm/<id>/robot.bin
/APM/nnm/<id>/policies/<nome>.nnm
```

## Script

| Script | Ruolo |
|---|---|
| `tools/robots/zero_policy_step.py` | Carica MJCF e fa un passo a policy zero |
| `tools/robots/train_velocity.py` | Entrypoint ricetta MjLab (job GPU) |
| `tools/robots/export_nnm.py` | ONNX → `.nnm` int8 con `robot_id` |
| `tools/robots/pack_robot_bin.py` | `profile.json` → `robot.bin` |
| `tools/export_policy_c.py` | ONNX → header float32 (fallback flash / parity) |

## Ordine di training suggerito

1. MicroDuck (già integrato)
2. Microban (MjLab già pronto)
3. Legolas, Upkie, Zeroth (dopo conversione MJCF e stand a policy zero)
4. Rex, Yertle (oggi non MuJoCo)
5. Bimo (8 giunti, 20 Hz)

## Contratto osservazione

Comune a tutti: gyro FLU, gravità FLU, `q−q0`, `q̇`, azione precedente, twist `vx,vy,wz`. Comandi testa/corpo solo se il robot li ha (vettore più corto, non imbottito a 61).
