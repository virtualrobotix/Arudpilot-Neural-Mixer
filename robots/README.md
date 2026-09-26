# robots/

Una cartella per robot. La stessa classificazione si ritrova sulla microSD dell'autopilota.

```
robots/<id>/
  robot/profile.json   topologia: giunti, q0, osservazione, frequenza, upstream, simulazione
  robot/ppo.yaml       architettura PPO e ambiente di training compatibile con ArduPilot
  robot/robot.bin      topologia per il firmware   -> /APM/nnm/<id>/robot.bin
  robot/scene.xml      scena MuJoCo generata da fetch_upstream.py (non versionata)
  policies/*.nnm       policy int8 di questo robot  -> /APM/nnm/<id>/policies/
  policies/README.md   elenco e provenienza delle policy
```

Tutti i file `profile.json`, `ppo.yaml`, le pagine `docs/robots/*.md` e i `policies/README.md` si
rigenerano da [`tools/robots/catalog.py`](../tools/robots/catalog.py):

```bash
.venv/bin/python tools/robots/build_catalog.py
```

Catalogo e pagine per robot: [docs/robots/README.md](../docs/robots/README.md).
Training: [docs/robots/training.md](../docs/robots/training.md).
