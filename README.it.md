# ArduPilot Neural Mixer (AP_NNMixer)

[English](README.md) | **Italiano**

*Developed by Roberto Navoni — DelphyAI LAB · r.navoni74@gmail.com*

AP_NNMixer esegue una policy di locomozione appresa per rinforzo (PPO) **dentro ArduPilot**, come task del
firmware ArduRover. La rete legge IMU, RC e modalità di volo di ArduPilot e comanda i giunti del robot
attraverso le uscite servo di ArduPilot. Nessun computer di bordo aggiuntivo: la policy è C puro nel
firmware e i pesi si caricano dalla microSD.

Il robot di riferimento è [MicroDuck](docs/robots/microduck.md) (Pollen Robotics, 14 servo). Lo stesso
firmware e la stessa pipeline di training coprono un [catalogo di bipedi e quadrupedi](docs/robots/README.md).

```
 GCS / MAVProxy ──MAVLink──▶ ArduRover ─ RC, modi ─▶ AP_NNMixer (PPO, 50 Hz) ─▶ SRV_Channels ─▶ giunti
                             AP_InertialSensor ───▶ filtro gravità + obs      ◀── feedback giunti
                             microSD /APM/nnm/<robot>/robot.bin + policies/*.nnm  → 2 slot int8 in RAM
```

Video demo: [`docs/media/demo_mavproxy_mlp.mp4`](docs/media/demo_mavproxy_mlp.mp4) (MAVProxy → ArduPilot →
rete → MuJoCo). Hardware-in-the-loop su Pixhawk 6C: [`docs/media/pixhawk6c_mlp_battery_imu.mp4`](docs/media/pixhawk6c_mlp_battery_imu.mp4).

## Cosa funziona oggi

| | Stato |
|---|---|
| MicroDuck in SITL con la pianta MuJoCo | sta in piedi, cammina, gira; batteria HIL 8/8 PASS |
| MicroDuck su Pixhawk 6C Mini (hardware-in-the-loop, build float32) | 15 s in piedi, forward p50 4,9 ms, CPU 32,8% |
| Policy da microSD, due slot int8, switch con fusione di 0,5 s | SITL: batteria 8/8 PASS con `walk.nnm` da SD (parità firmware vs `.nnm` 2,7e-7), test di switch 5/5; Pixhawk 6C: compila, non ancora provato sulla scheda |
| Twist da stick in MANUAL, HOLD, GUIDED / AUTO / RTL / SMART_RTL | nel firmware |
| Altri robot | profili, configurazioni PPO, scene MuJoCo; policy Microban convertita — vedi [il catalogo](docs/robots/README.md) |
| Robot reale senza simulatore | serve un backend bus servo (Dynamixel / Feetech) per il feedback giunti, non ancora scritto |

---

## Quickstart 1 — firmware su un autopilota

Scheda provata: **Pixhawk 6C Mini** (STM32H743, flash 2 MB, microSD). Clonare in un percorso **senza
spazi**: la build ChibiOS fallisce con gli spazi nel percorso.

```bash
git clone --recurse-submodules https://github.com/virtualrobotix/Ardupilot-Neural-Mixer.git ~/nnmixer
cd ~/nnmixer
uv venv --python 3.12 && uv pip install -r requirements.txt
cd ardupilot
../.venv/bin/python ./waf configure --board Pixhawk6C-NNMixer
../.venv/bin/python ./waf rover --upload
```

Senza `--upload`, si carica `build/Pixhawk6C-NNMixer/bin/ardurover.apj` con QGroundControl o Mission Planner
(firmware personalizzato). Due target:

| Target | Flash usata | Contenuto |
|---|---:|---|
| `Pixhawk6C-NNMixer` | 1.947.204 B (18,9 KB liberi) | policy da SD + MLP float32 di MicroDuck in flash come riserva |
| `Pixhawk6C-NNMixerSD` | 1.154.888 B (811 KB liberi) | solo policy da SD; spazio per altre funzioni ArduPilot |

Altre schede ArduPilot: un H7 con microSD e circa 500 KB di heap libero. Si crea
`libraries/AP_HAL_ChibiOS/hwdef/<Scheda>-NNMixer/hwdef.dat` con `include ../<Scheda>/hwdef.dat` e
`define AP_NNMIXER_ENABLED 1` (più `define AP_NNMIXER_BAKED_MLP_ENABLED 0` se la flash è stretta).

**microSD.** Una cartella per robot, dentro solo le policy di quel robot:

```
/APM/nnm/microduck/robot.bin
/APM/nnm/microduck/policies/walk.nnm
```

Si copiano da [`sitl/APM/nnm/`](sitl/APM/nnm/) oppure da `robots/<id>/robot/robot.bin` e
`robots/<id>/policies/*.nnm`.

**Parametri** (poi riavvio):

| Parametro | Valore | Perché |
|---|---|---|
| `NNM_ENABLE` | 1 | attiva il task |
| `NNM_ROBOT` | indice del robot nel [catalogo](docs/robots/README.md) (MicroDuck 0) | topologia, letta al boot |
| `NNM_POLICY` | 0 | indice del file policy in ordine alfabetico; si cambia a caldo |
| `SERVO1..14_FUNCTION` | 94..107 | uscite dei giunti (Scripting1..14) |
| `INS_GYRO_FILTER` | 0 | la policy vuole il gyro grezzo; i 4 Hz di default di Rover fanno cadere il bipede |
| `SCHED_LOOP_RATE` | 200 | filtro di gravità a 200 Hz, come in training |

[`sitl/nnmixer.parm`](sitl/nnmixer.parm) contiene l'insieme completo (su hardware si saltano le righe `SIM_*`).

**Prima accensione** (robot sospeso). Al boot la GCS mostra `NNMixer: robot microduck joints=14 obs=61` e
`NNMixer: loaded .../walk.nnm into slot 0`. Telemetria: `PPO_FAIL` 0 ok, 1 disarmato, 2 nessun feedback
giunti, 3 feedback vecchio; `PPO_PGZ` ≈ −1 in piedi; `PPO_SLOT` slot attivo. Oggi il feedback dei giunti
arriva dal ponte MuJoCo via USB:

```bash
.venv/bin/mjpython scripts/hil_mavlink_mujoco.py --port /dev/cu.usbmodem11201 --configure --seconds 30
```

---

## Quickstart 2 — SITL con MuJoCo (senza hardware)

```bash
cd ardupilot && ../.venv/bin/python ./waf configure --board sitl && ../.venv/bin/python ./waf rover && cd ..
.venv/bin/python scripts/demo_mavproxy.py            # finestra MuJoCo + SITL + sessione MAVProxy scriptata
```

La demo arma, sta in piedi, cammina avanti, indietro, di lato, gira di 90°, passa in HOLD e disarma.
`scripts/run_sitl.sh` copia `sitl/APM/nnm/` dove il SITL la vede come microSD. Sessione manuale e batteria
di test automatica: [documento di riferimento §8](docs/reference/microduck-ppo.it.md#8-demo-e-test).
`scripts/policy_switch_test.py` verifica lo switch a due slot (rimandato mentre cammina, eseguito in HOLD)
e il rifiuto del file di un altro robot.

---

## Robot supportati

La topologia si sceglie al boot (`NNM_ROBOT`); le policy si cambiano a caldo, solo fra quelle di quel
robot. Tutti i robot usano la rete di MicroDuck (MLP 512-256-128 ELU, pesi int8); cambiano solo le
dimensioni di ingresso e uscita.

| `NNM_ROBOT` | Robot | Tipo | Giunti | Obs | Policy nel repo |
|---:|---|---|---:|---:|---|
| 0 | [MicroDuck](docs/robots/microduck.md) | bipede | 14 | 61 | `walk.nnm` |
| 1 | [Microban](docs/robots/microban.md) | umanoide | 18 | 63 | `walk.nnm` (upstream, da rifinire) |
| 2 | [Zeroth-01](docs/robots/zeroth.md) | umanoide | 20 | 69 | — |
| 3 | [Bimo](docs/robots/bimo.md) | bipede | 8 | 33 | — |
| 4 | [Legolas](docs/robots/legolas.md) | bipede | 10 | 39 | — |
| 5 | [Upkie](docs/robots/upkie.md) | bipede a ruote | 6 | 27 | — |
| 6 | [Rex / SpotMicro](docs/robots/rex.md) | quadrupede | 12 | 45 | — |
| 7 | [Yertle](docs/robots/yertle.md) | quadrupede | 12 | 45 | — |

Ogni pagina linka il repo originale, il file del modello, CAD e BOM, ed elenca giunti, `q0`, uscite servo,
il file di architettura PPO (`robots/<id>/robot/ppo.yaml`) e le policy disponibili.

## Addestrare una policy che si installa così com'è

L'ambiente MuJoCo è già il deployment ArduPilot: fisica al loop rate dell'autopilota, gravità dal filtro IMU
del firmware, azione tagliata e codificata in PWM come nel firmware, osservazione ordinata dal profilo del
robot. L'actor si addestra sulla griglia int8 dalla prima iterazione, quindi il file `.nnm` è la rete
addestrata, senza quantizzazione successiva.

```bash
.venv/bin/python tools/robots/fetch_upstream.py --robot rex             # repo originale + scena MuJoCo
.venv/bin/python tools/robots/train_velocity.py --robot rex --name walk # PPO int8 sul contratto di deployment
.venv/bin/python tools/robots/train_velocity.py --robot rex --eval robots/rex/policies/walk.nnm
```

Dettagli, elenco dei comportamenti del firmware replicati e ricetta GPU (mjlab):
[docs/robots/training.md](docs/robots/training.md).

---

## Come funziona

- **Task a 50 Hz.** Osservazione nel frame FLU del tronco (gyro, gravità, posizioni e velocità dei giunti,
  azione precedente, twist), forward in C puro, azione tagliata, `q_target = q0 + azione`, PWM
  `1500 + q/0,003 µs`.
- **Gravità** da un piccolo filtro complementare solo-IMU al loop rate, non dall'EKF.
- **Robot e policy.** `robot.bin` fissa al boot giunti, dimensione dell'osservazione, `q0`, prima uscita servo
  e frequenza. I file `.nnm` portano l'id del robot; un file di un altro robot viene rifiutato.
- **Due slot in RAM.** La policy attiva si copia dalla SD in RAM e gira dalla RAM. Uno switch carica l'altro
  slot mentre la policy corrente continua, poi fonde le due uscite per `NNM_BLEND_MS` (500 ms). Gli switch
  sono accettati da disarmato o in HOLD.
- **Modi.** MANUAL: stick. HOLD: twist zero. GUIDED / AUTO / RTL / SMART_RTL: velocità e velocità di
  rotazione desiderate da Rover, saturate su `NNM_VX_MAX` / `NNM_WZ_MAX`. Disarmato: servo a riposo.

La storia completa di MicroDuck — training, contratto di osservazione, rete, export in C, test di parità,
protocollo SITL, modifiche ad ArduPilot e risultati — è nel [documento di riferimento](docs/reference/microduck-ppo.it.md).

## Documentazione

| Documento | Contenuto |
|---|---|
| [docs/reference/microduck-ppo.it.md](docs/reference/microduck-ppo.it.md) ([EN](docs/reference/microduck-ppo.md)) | integrazione MicroDuck in dettaglio, risultati, glossario |
| [docs/robots/README.md](docs/robots/README.md) | configurazioni dei robot e pagine per robot |
| [docs/robots/training.md](docs/robots/training.md) | training MuJoCo compatibile con ArduPilot, int8 |
| [docs/architettura-integrazione.pdf](docs/architettura-integrazione.pdf) / [.pptx](docs/architettura-integrazione.pptx) | slide dell'architettura |
| [docs/progetto-nnmixer-ardupilot-ppo.md](docs/progetto-nnmixer-ardupilot-ppo.md) | documento di progetto |

## Struttura del repository

| Percorso | Cosa |
|---|---|
| `ardupilot/` | submodule: [virtualrobotix/ardupilot](https://github.com/virtualrobotix/ardupilot/tree/microduck-ppo), branch `microduck-ppo` |
| `ardupilot/libraries/AP_NNMixer/` | il task, il caricatore da SD, forward int8 e float32 |
| `ardupilot/libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6C-NNMixer*/` | target Pixhawk 6C |
| `robots/<id>/robot/` | `profile.json` (topologia), `ppo.yaml` (PPO + ambiente), `robot.bin`, `scene.xml` |
| `robots/<id>/policies/` | policy `.nnm` di quel robot (e `.onnx` di origine) |
| `sitl/APM/nnm/` | struttura microSD usata dal SITL |
| `tools/robots/` | catalogo, download upstream, ambiente a contratto, training int8, export, `robot.bin` |
| `tools/` | export ONNX → C di MicroDuck, test di parità, rollout di riferimento |
| `plant/mujoco_json_plant.py` | pianta MuJoCo per il SITL (SIM_JSON) |
| `scripts/` | avvio SITL e pianta, demo, batteria HIL, ponte HIL Pixhawk |
| `tests/` | `test_nnm_pipeline.py`: training int8 → `.nnm` → forward C del firmware |
