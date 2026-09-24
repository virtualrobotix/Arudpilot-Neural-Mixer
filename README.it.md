# MicroDuck su ArduPilot — una policy PPO di locomozione come task di ArduRover, MuJoCo come pianta SITL

[English](README.md) | **Italiano**

*Developed by Roberto Navoni — DelphyAI LAB · r.navoni74@gmail.com*

Questo repository porta un controllore di camminata appreso per rinforzo per il
[MicroDuck](https://huggingface.co/spaces/pollen-robotics/microduck-simulator) (bipede da 25 cm con 14 servo,
Pollen Robotics / Hugging Face) **dentro ArduPilot**. La rete neurale gira come task a 50 Hz del firmware
ArduRover, legge solo i sensori di ArduPilot e comanda i 14 giunti attraverso le uscite servo di ArduPilot.
In simulazione il corpo del robot è un modello MuJoCo collegato al SITL di ArduPilot come farebbe un motore
fisico esterno (Gazebo, RealFlight). Lo si comanda da MAVProxy con gli stick, arm/disarm e le modalità.

Non si dà nulla per scontato: il [glossario](#glossario) in fondo definisce ogni termine usato.

```
 MAVProxy / GCS  ──MAVLink (tcp:5760)──▶  ArduRover SITL (un solo binario)              ──SIM_JSON udp:9002──▶  pianta MuJoCo
   arm, stick rc,                           RC_Channels ─▶ AP_MicroDuck (PPO, 50 Hz) ─▶ SRV_Channels (14 PWM)      14 XL330 (modello BAM)
   mode HOLD, param                         AP_InertialSensor ◀── backend SITL JSON ◀────udp:9003 (imu, quat, giunti)────┘
```

Video demo (68 s, script MAVProxy → ArduPilot → rete → MuJoCo, con i comandi MAVLink mostrati a schermo):
[`docs/media/demo_mavproxy_mlp.mp4`](docs/media/demo_mavproxy_mlp.mp4).
Slide dell'architettura: **IT** [`docs/architettura-integrazione.pdf`](docs/architettura-integrazione.pdf) / [`.pptx`](docs/architettura-integrazione.pptx) ·
EN [`docs/architecture-integration.pdf`](docs/architecture-integration.pdf) — entrambe generate da `tools/build_architecture_slides.py`.

---

## Indice

1. [Il problema che risolve](#1-il-problema-che-risolve)
2. [Come è stata addestrata la rete (il file `.pt`)](#2-come-è-stata-addestrata-la-rete-il-file-pt)
3. [Le due reti: MLP e Cartan](#3-le-due-reti-mlp-e-cartan)
4. [Da `.pt` al codice C nel firmware](#4-da-pt-al-codice-c-nel-firmware)
5. [Architettura dell'integrazione](#5-architettura-dellintegrazione)
6. [Cosa è stato modificato in ArduPilot](#6-cosa-è-stato-modificato-in-ardupilot)
7. [Setup](#7-setup)
8. [Demo e test](#8-demo-e-test)
9. [Risultati](#9-risultati)
10. [Struttura del repository](#10-struttura-del-repository)
11. [Glossario](#glossario)

---

## 1. Il problema che risolve

Un autopilota classico controlla un rover con anelli PID su velocità e direzione. Un bipede non si bilancia
così: la mappa fra sensori e le 14 posizioni servo che tengono in piedi il robot mentre cammina va appresa,
non scritta a mano. Il controllore appreso è una rete neurale chiamata **policy**.

La domanda a cui risponde questo repo: *quella policy può vivere dentro ArduPilot, usando lo stack sensori,
RC e servo di ArduPilot, così che lo stesso firmware giri prima in simulazione e poi sul flight controller
del robot reale?* La risposta è sì, e il repository è la prova funzionante.

Tre principi tenuti fermi:

- **Un solo firmware.** Nessun companion computer, nessun secondo autopilota. La policy è un task dello scheduler di ArduRover.
- **Solo API ArduPilot.** Il task legge `AP_InertialSensor` (IMU), `RC_Channels` (stick), una sorgente di
  feedback dei giunti, e scrive `SRV_Channels`. Non sa che MuJoCo esiste. Sostituire il simulatore con IMU e
  bus servo reali non cambia una riga del task.
- **La rete è C puro.** Niente ONNX Runtime né Python nel firmware. I pesi sono array `const float`, il forward
  è ~100 righe di C, identico sul Mac (SITL) e su un STM32.

---

## 2. Come è stata addestrata la rete (il file `.pt`)

L'addestramento **non** è avvenuto in questo repository: è stato fatto con lo stack ufficiale Pollen Robotics
([`pollen-robotics/microduck_rl`](https://github.com/pollen-robotics/microduck_rl)) e documentato nei quaderni
del laboratorio NOESIS EXPERIMENT. Lo spieghiamo qui perché il firmware ha senso solo se si sa cosa la rete si aspetta.

### 2.1 Apprendimento per rinforzo in un paragrafo

Il robot viene simulato migliaia di volte in parallelo su GPU. A ogni passo di controllo (50 volte al secondo)
la rete riceve un vettore di numeri che descrive lo stato del robot (l'**osservazione**), produce 14 numeri
(l'**azione**: dove deve andare ogni servo), il simulatore avanza di 20 ms e viene calcolata una **ricompensa**
(reward): positiva se cammina alla velocità comandata e resta in piedi, negativa se cade, se le azioni sono
brusche, se i giunti vanno a fine corsa. L'algoritmo di apprendimento, **PPO** (Proximal Policy Optimization),
aggiusta i pesi in modo che le azioni che portano a più ricompensa nel lungo periodo diventino più probabili.
Dopo abbastanza iterazioni la rete ha "scoperto" un'andatura. Nessuno l'ha programmata.

### 2.2 Il simulatore di addestramento

| Voce | Valore |
|---|---|
| Fisica | **MuJoCo Warp** (MuJoCo su GPU) via `mjlab` 1.3.0 |
| Modello robot | MJCF del MicroDuck da Onshape (14 giunti, trunk libero) |
| Attuatori | modello **BAM M6** del servo Dynamixel **XL330**: controllo in tensione, attrito misurato, limite di corrente, calo batteria |
| Passo fisico | 5 ms (200 Hz); la policy agisce ogni 4 passi → **50 Hz** |
| Ambienti paralleli | **2048** robot simulati insieme |
| Episodio | 20 s (1000 passi di controllo), finisce prima se il robot cade |
| Hardware | una NVIDIA RTX 3090 (Vast.ai) |

### 2.3 Cosa vede e cosa fa la rete — il contratto

È l'interfaccia che il firmware deve riprodurre **esattamente**. La policy è stata addestrata su questi 61
numeri, in quest'ordine e con queste unità; qualunque altra cosa è un altro mondo per la rete.

| Indice | Dim | Grandezza | Unità | Frame |
|---|---:|---|---|---|
| `[0:3]` | 3 | velocità angolare del trunk (gyro) | rad/s | trunk FLU (x avanti, y sinistra, z su) |
| `[3:6]` | 3 | direzione della gravità | vettore unitario (in piedi ≈ `[0,0,-1]`) | trunk FLU |
| `[6:20]` | 14 | posizione giunto meno posa di default `q0` | rad | ordine giunti sotto |
| `[20:34]` | 14 | velocità giunto | rad/s | |
| `[34:48]` | 14 | azione precedente (quella inviata ai servo al passo prima) | rad | |
| `[48:51]` | 3 | twist comandato `vx, vy` (m/s), `ωz` (rad/s) | SI | |
| `[51:55]` | 4 | comando testa (pitch collo/testa, yaw, roll) | rad | 0 in questo repo |
| `[55:61]` | 6 | comando corpo (x, y, z, roll, pitch, yaw) | m, rad | 0 in questo repo |

Azione: 14 offset in radianti; il target servo è `q_target = q0 + azione`.
Ordine giunti: `left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle, neck_pitch, head_pitch,
head_yaw, head_roll, right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle`.
Posa di default `q0` (rad): `[0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0, 0, 0, 0.0873, 0.4579, 0.0049, -0.4530]`.

Range dei comandi visti in training: `vx ∈ [-0.4, 0.4]` m/s, `vy ∈ [-0.3, 0.3]` m/s, `ωz ∈ [-1, 1]` rad/s.
Comando zero = stare fermi in piedi (addestrato esplicitamente).

### 2.4 La ricompensa (cosa vuol dire "bene")

Pesi dalla configurazione di training (`params/env.yaml` dei checkpoint):

| Termine | Peso | Significato |
|---|---:|---|
| `track_linear_velocity` | +2.0 | seguire `vx, vy` comandati |
| `track_angular_velocity` | +2.0 | seguire `ωz` comandato |
| `upright` | +2.0 | trunk verticale |
| `head_pose_tracking` | +2.0 | testa dove comandata |
| `pose` | +1.0 | restare vicini alla postura di default |
| `dof_pos_limits` | −1.0 | non spingere i giunti a fine corsa |
| `action_rate_l2` | 0 → −0.8 (curriculum) | azioni lisce, introdotto gradualmente |
| `body_ang_vel`, `angular_momentum` | −0.05, −0.02 | non dimenarsi |

Terminazione: l'episodio finisce quando il trunk cade (`fell_over`). A fine training circa metà degli episodi
termina ancora con una caduta entro 20 s sotto le randomizzazioni più dure: normale per un bipede di questa
taglia, ed è il motivo per cui il tracking di velocità non è perfetto.

### 2.5 Domain randomization (perché la policy sopravvive a un simulatore diverso)

Ogni episodio randomizza: attrito dei piedi, tensione batteria (6.5–8.2 V), budget di attrito dei servo,
armature dei giunti, massa/inerzia e baricentro di trunk e testa, bias degli encoder, disallineamento IMU,
rumore dei sensori e ritardo 0–1 passi sul gyro, più spinte casuali. È ciò che permette alla rete addestrata in
MuJoCo Warp di controllare la pianta MuJoCo su CPU di questo repo — e, dopo, il robot reale.

### 2.6 Impostazioni PPO e durata

| Voce | Valore |
|---|---|
| Algoritmo | PPO, implementazione `rsl_rl` (`OnPolicyRunner`) |
| Campioni raccolti per iterazione | 24 per ambiente × 2048 ambienti = **49 152** |
| **Epoche** di apprendimento per iterazione | 5 passate su quei campioni, in 4 mini-batch |
| **Iterazioni** | **2000** → 98,3 milioni di passi simulati ≈ 23 giorni di tempo-robot |
| Learning rate | 1e-3, adattivo su KL target 0.01 |
| Sconto γ / GAE λ | 0.99 / 0.95 |
| Clip / bonus entropia | 0.2 / 0.01 |
| Normalizzazione osservazioni | attiva (media/dev. std. correnti, vedi §4) |
| Checkpoint | `model_<iterazione>.pt` ogni 250 iterazioni; quello deployato è `model_1999.pt` |
| Tempo macchina | MLP ≈ 1,6 s/iterazione ≈ **55 min**; Cartan ≈ 2,2 s/iterazione ≈ **73 min** |
| Reward medio finale / lunghezza episodio | MLP 105 / 913 passi; Cartan 99 / 920 passi (su 1000) |

Due parole spesso confuse: una **iterazione** è un ciclo "raccogli 49 152 campioni, poi impara da loro";
un'**epoca** è una passata su quei campioni dentro la fase di apprendimento (5 per iterazione). Le iterazioni
sono 2000, quindi le epoche 10 000, ma "epoca" qui è il numero meno utile.

### 2.7 Cosa c'è dentro `model_1999.pt`

Un checkpoint PyTorch: l'**actor** (la policy che gira sul robot), il **critic** (una seconda rete che stima la
ricompensa futura — serve solo in training), lo stato dell'ottimizzatore e il **normalizzatore delle
osservazioni** (61 medie e 61 deviazioni standard accumulate in training). Si deployano solo actor e normalizzatore.

---

## 3. Le due reti: MLP e Cartan

Stesso task, stessa ricompensa, stesso budget 2048 × 2000, stessi seed. Cambia solo la classe di rete
dentro actor e critic.

### 3.1 MLP (`MDK_POLICY 0`) — il riferimento

Un percettrone multistrato classico: `61 → 512 → 256 → 128 → 14`, attivazione **ELU** fra gli strati.
197 896 parametri (773 KB in float32). È quello delle policy ufficiali Pollen.

### 3.2 Cartan (`MDK_POLICY 1`) — la sperimentale

Una **Cartan Network** ([arXiv:2505.24353](https://arxiv.org/abs/2505.24353)) con attivazione **DiLU**.
Invece di impilare strati lineari, ogni strato lavora su un punto di un gruppo di Lie risolubile (uno spazio
iperbolico in "coordinate di Cartan": uno scalare `c` più un vettore di 192 detto *fiber* o *paint*). Uno
strato fa: mappa lineare sul fiber, traslazione di gruppo (`beta`), *rotazione del fiber* con un vettore
unitario (`theta`) — operazione non lineare con `exp`, `log` e un prodotto scalare — poi DiLU
`(ELU(x) + 0.1x)/1.1` sul fiber. La lettura finale moltiplica il fiber per `exp(c)` e applica uno strato lineare.

Forma: `61 → 192 (embedding) → 3 × CartanLinear(193) → 14`. 127 054 parametri (496 KB float32, **−36 %**
rispetto all'MLP) e meno moltiplicazioni per passo. In laboratorio è competitiva sui task geometrici e sui
quadrupedi; sul MicroDuck a 2000 iterazioni l'MLP ha ancora reward migliore (105 vs 99) e tracking migliore,
mentre Cartan è più leggera e più veloce da valutare (0,25 ms vs 0,37 ms per passo in questo SITL). Entrambe
stanno in piedi e camminano in questo repo.

---

## 4. Da `.pt` al codice C nel firmware

1. **Export in ONNX** (in `microduck_rl`, `scripts/export.py`): traccia `actor(normalizer(obs))`. Il
   normalizzatore è *cotto* nel grafo: i primi due nodi ONNX sono `Sub(mean)` e `Div(std)`. Per questo il
   firmware non calcola mai medie o deviazioni: sono costanti apprese in training, come i pesi.
2. **ONNX → header C** (`tools/export_policy_c.py` per l'MLP, `tools/export_cartan_c.py` per Cartan): legge
   gli initializer e scrive `policy_mlp.h` / `policy_cartan.h` con `mean[61]`, `std[61]`, tutte le matrici dei
   pesi come `static const float` e `q0`.
3. **Forward in C** (`tools/microduck_infer.c`, copiato tal quale in `libraries/AP_MicroDuck/`):
   `microduck_forward()` per l'MLP e `microduck_cartan_forward()` per Cartan. Float32, nessun heap, solo `expf` e `logf`.
4. **Test di parità**: `tools/parity_check.py` passa 2201 osservazioni in ONNX Runtime e nel binario C —
   differenza massima 3,8e-6 (MLP), 6,0e-5 (Cartan). `tools/log_parity.py` fa lo stesso con le osservazioni
   che il *firmware* ha loggato durante una corsa, contro le azioni che il firmware ha davvero inviato
   (max 2,5e-7). La rete nel firmware è la rete addestrata in laboratorio.

---

## 5. Architettura dell'integrazione

Tre processi in simulazione; su hardware il terzo è il robot.

| Processo | Ruolo | Parla con |
|---|---|---|
| **MAVProxy** (o qualunque GCS, o `scripts/hil_test.py`) | operatore: arm/disarm, stick (`rc N pwm`), modi, parametri, telemetria | ArduRover via MAVLink, TCP 5760 |
| **ArduRover SITL** (`ardupilot/build/sitl/bin/ardurover --model JSON`) | il firmware: sensori, RC, modi, arming, task **AP_MicroDuck**, uscite servo | MAVProxy (MAVLink); pianta (SIM_JSON UDP 9002 out / 9003 in) |
| **Pianta MuJoCo** (`plant/mujoco_json_plant.py`) | il corpo del robot: 14 attuatori BAM-XL330, fisica, IMU, encoder | ArduRover (SIM_JSON) |

### 5.1 Il loop a 50 Hz dentro ArduRover

1. `AP_InertialSensor` fornisce gyro e accelerometro nel frame body di ArduPilot (FRD: x avanti, y destra, z giù).
2. `AP_MicroDuck::update_attitude()` (a loop rate) mantiene la **direzione della gravità** con un piccolo filtro
   complementare IMU-only: propagazione col gyro, correzione lenta con l'accelerometro (`MDK_ATT_TAU`). L'EKF
   *non* entra nell'osservazione, perché il training ha usato l'IMU grezza.
3. Posizioni e velocità dei giunti arrivano dalla pianta (in SITL) o arriveranno dal bus Dynamixel (su hardware).
4. Stick → twist in unità SI: `vx = stick2 × MDK_VX_MAX`, `vy = stick1 × MDK_VY_MAX`, `ωz = stick4 × MDK_WZ_MAX`.
   Il modo HOLD forza il twist a zero; disarmato forza l'azione a zero.
5. Il vettore di 61 viene assemblato nel **frame trunk FLU** (gyro `(x, −y, −z)`, gravità `(x, −y, −z)`), gira
   il forward C (≈0,3 ms), le 14 azioni vengono limitate e salvate come "azione precedente" per il tick dopo.
6. `q_target = q0 + azione` → PWM `1500 + q_target/0.003` (1 µs = 3 mrad) → funzioni `SRV_Channels`
   Scripting1..14 → in SITL il pacchetto servo JSON, su hardware il bus servo.

### 5.2 Il protocollo della pianta (SIM_JSON)

Il SITL di ArduPilot ha un backend fisico "JSON" standard: invia un pacchetto binario con 16 PWM a
`SIM_RATE_HZ` e aspetta una riga JSON con `timestamp, imu{gyro, accel_body}, position, velocity, quaternion`
(lock-step: la simulazione non può correre avanti al firmware). Questo repo aggiunge un campo opzionale,
`joints{jpos[14], jvel[14]}`, che `SIM_JSON.cpp` scrive in `sitl->state`: è l'implementazione SITL della
sorgente di feedback dei giunti. `SIM_RATE_HZ 200` e `timestep 0.005` in MuJoCo danno un passo fisico per
frame, e il task a 50 Hz vede 4 sotto-passi, come in training.

Frame: MuJoCo è z-up / trunk FLU, ArduPilot è NED / FRD. La pianta converte con `(x, y, z) → (x, −y, −z)` per i
vettori e `(w, x, y, z) → (w, x, −y, −z)` per il quaternione.

---

## 6. Cosa è stato modificato in ArduPilot

Fork [`virtualrobotix/ardupilot`](https://github.com/virtualrobotix/ardupilot/tree/microduck-ppo), branch
`microduck-ppo`, sopra `master` upstream (24 set 2026). Due commit, 7 file toccati + 1 libreria nuova.

| File | Modifica |
|---|---|
| `libraries/AP_MicroDuck/` (nuova) | `AP_MicroDuck.{h,cpp}`: osservazione, filtro gravità, storia azioni, stick, uscite servo, parametri `MDK_*`, log `MDK/MDKQ/MDKV/MDKA`, `NAMED_VALUE_FLOAT PPO_*`; `microduck_infer.{h,c}`; `policy_mlp.h`, `policy_cartan.h` |
| `libraries/SITL/SIM_JSON.{h,cpp}` | parsing di `joints/jpos`, `joints/jvel` (tipo `DATA_FLOAT_ARRAY14`) |
| `libraries/SITL/SITL.h` | `sitl_fdm.joint_pos/joint_vel/joint_count/joint_time_us` |
| `Rover/Parameters.{h,cpp}` | `g2.microduck`, gruppo `MDK_` (indice 63) |
| `Rover/Rover.cpp` | scheduler: `update_attitude` a loop rate, `update` a 50 Hz |
| `Rover/wscript` | link di `AP_MicroDuck` |

Parametri (`sitl/microduck.parm` li imposta per il SITL): `MDK_ENABLE`, `MDK_POLICY` (0 MLP / 1 Cartan),
`MDK_VX_MAX 0.4`, `MDK_VY_MAX 0.3`, `MDK_WZ_MAX 1.0`, `MDK_WD_MS 40` (watchdog feedback giunti),
`MDK_ATT_SRC` (0 filtro IMU / 1 AHRS, debug), `MDK_ATT_TAU 0.5`, `MDK_RC_VX/VY/WZ 2/1/4`, `MDK_ACT_MAX 2.0`,
`MDK_LOG`, `MDK_HOLD_MODE 4`, `MDK_SRV_FN0 94`. Più `SERVO1..14_FUNCTION 94..107`, `SIM_RATE_HZ 200`,
`SCHED_LOOP_RATE 200` e **`INS_GYRO_FILTER 0`** (vedi §9).

Telemetria: `PPO_MS` tempo del forward, `PPO_PGZ` gravità z (in piedi ≈ −1), `PPO_VX` comando, `PPO_FAIL`
(0 ok, 1 disarmato, 2 nessun feedback giunti, 3 feedback vecchio, 4 errore forward).

---

## 7. Setup

macOS o Linux. Servono `uv` (o un Python 3.11/3.12), `ffmpeg` (video), `mavproxy.py`, una toolchain C.

```bash
git clone --recurse-submodules https://github.com/virtualrobotix/microduck-ap-ppo-sitl.git
cd microduck-ap-ppo-sitl
uv venv --python 3.12 && uv pip install -r requirements.txt
cd ardupilot && git submodule update --init --recursive --depth 1
../.venv/bin/python ./waf configure --board sitl && ../.venv/bin/python ./waf rover && cd ..
```

La pianta carica la scena MJCF del MicroDuck dal checkout di NOESIS EXPERIMENT (`plant/mujoco_json_plant.py`,
`NOESIS_MJCF`); con `MICRODUCK_MJCF` si indica la propria copia di `microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml`.

Rigenerare gli header C da altri checkpoint:

```bash
.venv/bin/python tools/export_policy_c.py  mio_mlp.onnx    --out ardupilot/libraries/AP_MicroDuck/policy_mlp.h    --name mlp
.venv/bin/python tools/export_cartan_c.py  mio_cartan.onnx --out ardupilot/libraries/AP_MicroDuck/policy_cartan.h --name cartan
.venv/bin/python tools/parity_check.py mio_mlp.onnx --name mlp            # deve stampare PARITY OK
```

---

## 8. Demo e test

Demo visiva in un comando (finestra MuJoCo + SITL + MAVProxy digitato dallo script):

```bash
.venv/bin/python scripts/demo_mavproxy.py                  # MLP
.venv/bin/python scripts/demo_mavproxy.py --policy 1       # Cartan
.venv/bin/python scripts/demo_mavproxy.py --video out.mp4 --keep
```

Sequenza digitata in MAVProxy: `param set MDK_POLICY`, `mode manual`, `rc all 1500`, `arm throttle` → 6 s in
piedi → `rc 2 2000` avanti 3 s → `rc 2 1000` indietro 2 s → `rc 1 1800` laterale 2 s → `rc 4 2000` finché la
velocità di imbardata ricevuta in `ATTITUDE` integra 90° → `rc 2 2000` avanti 5 s → `mode hold` → `mode manual`
→ `disarm`. L'overlay del video mostra ogni comando con il messaggio MAVLink che produce. Con `--keep` resta
tutto acceso per comandi a mano.

Manuale: terminale 1 `.venv/bin/mjpython plant/mujoco_json_plant.py` (macOS; `scripts/run_plant.sh` su Linux),
terminale 2 `scripts/run_sitl.sh --console`, poi gli stessi comandi MAVProxy a mano.

Batteria HIL automatica (SITL avviato headless, es. `scripts/run_sitl.sh --no-mavproxy`):

```bash
.venv/bin/python scripts/hil_test.py battery               # MLP
.venv/bin/python scripts/hil_test.py battery --policy 1    # Cartan
```

Passi e criteri: arm; stand 15 s (`PPO_FAIL 0`, `PPO_PGZ < −0.9`); avanti `rc 2 1750` 12 s (`PPO_VX ≈ 0.2`,
in piedi); laterale; rotazione; HOLD (`PPO_VX = 0`); disarm (`PPO_FAIL 1`); parità in-situ delle osservazioni
loggate contro l'ONNX.

Rollout di riferimento senza ArduPilot (solo policy ↔ pianta), utile per separare problemi di pianta da problemi di firmware:

```bash
.venv/bin/python tools/policy_rollout_plant.py policies/microduck_mlp_2048x2000_it1999.onnx --vx 0.3 --seconds 10
```

---

## 9. Risultati

| | MLP (`MDK_POLICY 0`) | Cartan (`MDK_POLICY 1`) |
|---|---|---|
| Parità C vs ONNX (offline, 2201 osservazioni) | max 3,8e-6 | max 6,0e-5 |
| Parità in-situ (obs loggate dal firmware → ONNX vs azioni firmware) | max 2,5e-7 | p99 1,3e-6 |
| Forward nel SITL (Apple Silicon, `-O2`) | ~0,37 ms | ~0,25 ms |
| Pesi in flash (float32) | 773 KB | 496 KB |
| Batteria HIL (arm, stand, avanti, laterale, rotazione, HOLD, disarm, parità) | 8/8 PASS | 8/8 PASS |
| Avanti a comando 0,3 / 0,4 m/s, 15 s ciascuno + rotazione | nessuna caduta, ~0,15–0,25 m/s effettivi | — |

**La lezione che è costata una mattinata.** Il primo HIL cadeva dopo un secondo pur con osservazioni e rete
perfette (parità esatta). Causa: ArduRover ha `INS_GYRO_FILTER` **4 Hz** di default (sensato per un veicolo a
ruote). Il gyro arrivava alla rete con decine di millisecondi di ritardo e ampiezza dimezzata. Con
`INS_GYRO_FILTER 0` (o 40 Hz) il duck sta in piedi e cammina. La stessa regola varrà su hardware: nessun
passa-basso lento sul gyro che alimenta la policy.

Il tracking di velocità (~0,15–0,25 m/s a comando 0,4) è identico al rollout Python di riferimento sulla stessa
pianta CPU: è il gap fra MuJoCo-CPU e il simulatore di training MuJoCo-Warp, non l'integrazione ArduPilot.

Prossimi passi: backend Dynamixel per il feedback giunti sul robot reale; forward misurato su STM32H7;
comandi twist da MAVLink GUIDED oltre agli stick.

---

## 10. Struttura del repository

| Path | Cosa |
|---|---|
| `ardupilot/` | submodule: fork, branch `microduck-ppo` |
| `ardupilot/libraries/AP_MicroDuck/` | il task, il forward C, gli header dei pesi generati |
| `plant/mujoco_json_plant.py` | pianta MuJoCo: protocollo SIM_JSON, attuatori BAM, frame, registrazione mp4 opzionale con overlay dei comandi |
| `policies/` | i due export ONNX validati (iterazione 1999) |
| `tools/export_policy_c.py`, `tools/export_cartan_c.py` | ONNX → header C |
| `tools/microduck_infer.{h,c}` | forward C (MLP e Cartan) |
| `tools/parity_check.py`, `tools/parity_check*.c` | ONNX Runtime vs C |
| `tools/log_parity.py` | log del firmware vs ONNX |
| `tools/policy_rollout_plant.py` | policy ↔ pianta senza ArduPilot |
| `sitl/microduck.parm` | parametri SITL |
| `scripts/run_plant.sh`, `scripts/run_sitl.sh` | avvio |
| `scripts/hil_test.py` | batteria automatica |
| `scripts/demo_mavproxy.py` | demo visiva pilotata via MAVProxy |
| `docs/` | documento di progetto (IT), video demo |

---

## Glossario

- **Policy** — la rete neurale che mappa osservazione → azione; il "controllore".
- **Actor / critic** — in PPO si addestrano due reti: l'actor è la policy; il critic stima quanta ricompensa
  seguirà da uno stato e serve solo ad addestrare l'actor.
- **Osservazione** — il vettore di numeri che la policy riceve a ogni passo (qui 61 float).
- **Azione** — il vettore che la policy produce (qui 14 offset di giunto in radianti).
- **Ricompensa (reward)** — il punteggio scalare calcolato dal simulatore a ogni passo; il training ne massimizza la somma.
- **Episodio** — un tentativo simulato, qui fino a 20 s; finisce prima se il robot cade.
- **Iterazione** — un ciclo PPO: raccogli campioni da tutti gli ambienti paralleli, poi impara da loro.
- **Epoca** — una passata sui campioni raccolti durante la fase di apprendimento (qui 5 per iterazione).
- **Domain randomization** — variare i parametri fisici a ogni episodio perché la policy non si adatti a un solo simulatore.
- **Normalizzazione delle osservazioni** — sottrarre una media corrente e dividere per una deviazione standard
  corrente ogni componente dell'osservazione; le statistiche vengono congelate all'export e cotte nella rete.
- **ONNX** — formato di scambio per reti neurali; qui il passo intermedio fra PyTorch e C.
- **`.pt`** — checkpoint PyTorch (pesi + ottimizzatore + normalizzatore).
- **PPO** — Proximal Policy Optimization, l'algoritmo di RL usato (implementazione `rsl_rl`).
- **MLP** — percettrone multistrato, la rete completamente connessa standard.
- **Cartan Network / DiLU** — la famiglia di reti alternativa di arXiv:2505.24353, che opera su un gruppo di Lie
  risolubile; DiLU è la sua attivazione.
- **SITL** — Software In The Loop: il firmware ArduPilot vero compilato per PC, con sensori simulati.
- **HIL** — Hardware In The Loop; qui usato in senso lato per "firmware nel loop con una pianta fisica esterna".
- **Pianta** — termine dell'ingegneria del controllo per il sistema fisico controllato (il corpo del robot).
- **SIM_JSON** — l'interfaccia fisica JSON generica di ArduPilot per simulatori esterni.
- **MAVLink / MAVProxy** — il protocollo di telemetria e la ground station a riga di comando di ArduPilot.
- **FRD / FLU / NED** — convenzioni degli assi: Forward-Right-Down (body ArduPilot), Forward-Left-Up (trunk
  MuJoCo), North-East-Down (mondo ArduPilot).
- **BAM** — Better Actuator Models: il modello misurato tensione/attrito del servo XL330 usato in training e nella pianta.
- **Lock-step** — il simulatore avanza solo quando il firmware ha consumato il frame precedente, così il tempo resta coerente.
