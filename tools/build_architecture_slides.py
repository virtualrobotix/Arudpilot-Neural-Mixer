#!/usr/bin/env python3
"""Build the architecture deck (IT) explaining how MAVProxy, ArduRover SITL, AP_MicroDuck and the MuJoCo
plant are connected, with the ArduRover integration spelled out. Output: docs/architettura-integrazione.pptx

    .venv/bin/python tools/build_architecture_slides.py
"""

from __future__ import annotations

from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "architettura-integrazione.pptx"
CREDIT = "Developed by Roberto Navoni : r.navoni74@gmail.com"

NAVY = RGBColor(0x0D, 0x3B, 0x66)
RED = RGBColor(0xD6, 0x27, 0x28)
GREY = RGBColor(0x55, 0x55, 0x55)
LIGHT = RGBColor(0xF4, 0xF7, 0xFB)
BLUE_FILL = RGBColor(0xE3, 0xEC, 0xF7)
RED_FILL = RGBColor(0xFD, 0xEC, 0xEA)
GREEN_FILL = RGBColor(0xE6, 0xF4, 0xEA)
GREEN = RGBColor(0x2E, 0x7D, 0x32)
ORANGE_FILL = RGBColor(0xFF, 0xF3, 0xE0)
ORANGE = RGBColor(0xE6, 0x7E, 0x22)

prs = None
BLANK = None
_n = 0
LANG = "it"


def tr(s):
    if LANG == "en":
        return TR.get(s, s)
    return s


# Italian source strings -> English. Anything not listed is language-neutral (identifiers, numbers).
TR = {
    "MicroDuck su ArduPilot": "MicroDuck on ArduPilot",
    "Come sono collegati MAVProxy, ArduRover, la rete PPO e MuJoCo": "How MAVProxy, ArduRover, the PPO network and MuJoCo are connected",
    "Roberto Navoni — DelphyAI LAB · r.navoni74@gmail.com · 24 settembre 2026": "Roberto Navoni — DelphyAI LAB · r.navoni74@gmail.com · 24 September 2026",
    "Un solo firmware ArduRover. La rete neurale (61 osservazioni → 14 servo, 50 Hz) è un task dello scheduler. MuJoCo è il corpo del robot, collegato al SITL come un motore fisico esterno. Il comando arriva da MAVProxy via MAVLink.":
        "One ArduRover firmware. The neural network (61 observations → 14 servos, 50 Hz) is a scheduler task. MuJoCo is the robot body, attached to the SITL like an external physics engine. Commands come from MAVProxy over MAVLink.",
    "1. Tre processi, due collegamenti": "1. Three processes, two links",
    "In simulazione: GCS, firmware, corpo del robot. Su hardware il terzo processo è il robot vero.": "In simulation: GCS, firmware, robot body. On hardware the third process is the real robot.",
    "MAVProxy (GCS)": "MAVProxy (GCS)",
    "operatore o script": "operator or script",
    "telemetria PPO_*": "PPO_* telemetry",
    "(un solo binario ardurover)": "(a single ardurover binary)",
    "RC_Channels → twist SI": "RC_Channels → twist in SI units",
    "AP_InertialSensor → gyro, gravità": "AP_InertialSensor → gyro, gravity",
    "AP_MicroDuck: rete PPO 50 Hz": "AP_MicroDuck: PPO network 50 Hz",
    "backend SIM_JSON": "SIM_JSON backend",
    "Pianta MuJoCo": "MuJoCo plant",
    "14 servo XL330 (modello BAM)": "14 XL330 servos (BAM model)",
    "fisica 200 Hz (5 ms)": "physics 200 Hz (5 ms)",
    "IMU, encoder giunti": "IMU, joint encoders",
    "Lock-step: il SITL invia i PWM e aspetta il JSON; MuJoCo fa un passo da 5 ms per frame (SIM_RATE_HZ 200). Il task PPO gira ogni 4 frame = 50 Hz, come in training.":
        "Lock-step: the SITL sends the PWM and waits for the JSON; MuJoCo takes one 5 ms step per frame (SIM_RATE_HZ 200). The PPO task runs every 4 frames = 50 Hz, as in training.",
    "Il task PPO non ha alcun collegamento con MuJoCo: vede solo API ArduPilot. Sostituire la pianta con IMU + bus Dynamixel reali non cambia una riga di AP_MicroDuck.":
        "The PPO task has no link to MuJoCo: it only sees ArduPilot APIs. Replacing the plant with a real IMU + Dynamixel bus does not change a line of AP_MicroDuck.",
    "2. Dentro ArduRover: il loop a 50 Hz di AP_MicroDuck": "2. Inside ArduRover: the 50 Hz loop of AP_MicroDuck",
    "Tutto ciò che la rete vede passa dalle librerie standard di ArduPilot.": "Everything the network sees goes through ArduPilot's standard libraries.",
    "ArduRover — scheduler (SCHED_LOOP_RATE 200)": "ArduRover — scheduler (SCHED_LOOP_RATE 200)",
    "q, q̇ (14) da sitl->state": "q, q̇ (14) from sitl->state",
    "stick 1/2/4 → vx vy ωz": "sticks 1/2/4 → vx vy ωz",
    "Storia": "History",
    "azione precedente (14)": "previous action (14)",
    "Filtro gravità": "Gravity filter",
    "frame trunk FLU": "trunk FLU frame",
    "mean/std + rete": "mean/std + network",
    "a applicata → storia (obs[34:48]) al tick successivo": "applied a → history (obs[34:48]) at the next tick",
    "update_attitude() a loop rate: propaga la direzione della gravità col gyro, la corregge con l'accelerometro (MDK_ATT_TAU 0.5 s)":
        "update_attitude() at loop rate: propagates the gravity direction with the gyro, corrects it with the accelerometer (MDK_ATT_TAU 0.5 s)",
    "update() a 50 Hz: obs → forward → clip → PWM": "update() at 50 Hz: obs → forward → clip → PWM",
    "Failsafe: disarmato, feedback giunti assente o vecchio > MDK_WD_MS → azione 0 = posa stand (anche nella storia)":
        "Failsafe: disarmed, joint feedback missing or older than MDK_WD_MS → action 0 = stand pose (also in the history)",
    "3. Percorso dei sensori: da MuJoCo all'osservazione": "3. Sensor path: from MuJoCo to the observation",
    "Frame diversi, unità diverse: la pianta parla ArduPilot (FRD/NED), il task parla training (FLU).": "Different frames, different units: the plant speaks ArduPilot (FRD/NED), the task speaks training (FLU).",
    "gyro/accel sito IMU (FLU)": "gyro/accel at the IMU site (FLU)",
    "quaternione trunk (z-up)": "trunk quaternion (z-up)",
    "qpos, qvel 14 giunti": "qpos, qvel of 14 joints",
    "→ INS simulata (gyro, accel)": "→ simulated INS (gyro, accel)",
    "(estensione di questo repo)": "(extension added by this repo)",
    "gravità: filtro → (x, −y, −z)": "gravity: filter → (x, −y, −z)",
    "Conversione dei frame (fatta dalla pianta)": "Frame conversion (done by the plant)",
    "vettori: (x, y, z)_MuJoCo → (x, −y, −z)_ArduPilot  — vale per gyro, accel, posizione, velocità": "vectors: (x, y, z)_MuJoCo → (x, −y, −z)_ArduPilot — gyro, accel, position, velocity",
    "quaternione: (w, x, y, z) → (w, x, −y, −z)": "quaternion: (w, x, y, z) → (w, x, −y, −z)",
    "in piedi: accel_body = (0, 0, −9.81) come ArduPilot si aspetta; gravità proiettata nel task = (0, 0, −1) come in training":
        "standing: accel_body = (0, 0, −9.81) as ArduPilot expects; projected gravity in the task = (0, 0, −1) as in training",
    "Lezione del primo test": "Lesson from the first test",
    "ArduRover ha INS_GYRO_FILTER 4 Hz di default: il gyro arrivava alla rete con decine di ms di ritardo → caduta in 1 s":
        "ArduRover defaults to INS_GYRO_FILTER 4 Hz: the gyro reached the network tens of ms late → fall within 1 s",
    "INS_GYRO_FILTER 0 (o 40) → stabile. Nessun passa-basso lento sul gyro che alimenta la policy, anche su hardware":
        "INS_GYRO_FILTER 0 (or 40) → stable. No slow low-pass on the gyro that feeds the policy, on hardware too",
    "4. Percorso dei comandi: da MAVProxy al twist": "4. Command path: from MAVProxy to the twist",
    "Ogni comando digitato in MAVProxy è un messaggio MAVLink standard; ArduPilot non è stato modificato qui.": "Every command typed in MAVProxy is a standard MAVLink message; ArduPilot was not modified here.",
    "Comando MAVProxy": "MAVProxy command", "Messaggio MAVLink": "MAVLink message", "Cosa fa in ArduRover": "What ArduRover does", "Effetto sulla rete": "Effect on the network",
    "PPO_FAIL 1→0, servo attivi, la policy tiene in piedi": "PPO_FAIL 1→0, servos live, the policy keeps it standing",
    "RC_CHANNELS_OVERRIDE chan2=2000 (da sysid 255)": "RC_CHANNELS_OVERRIDE chan2=2000 (from sysid 255)",
    "MDK_HOLD_MODE → twist forzato a 0: sta fermo": "MDK_HOLD_MODE → twist forced to 0: stands still",
    "prossimo tick usa la rete Cartan": "next tick uses the Cartan network",
    "azione 0, storia azzerata, PWM 0 (pianta: idle)": "action 0, history cleared, PWM 0 (plant: idle)",
    "Telemetria di ritorno (NAMED_VALUE_FLOAT ogni 0.5 s): PPO_MS tempo del forward, PPO_PGZ gravità z (in piedi ≈ −1), PPO_VX comando, PPO_FAIL stato.":
        "Return telemetry (NAMED_VALUE_FLOAT every 0.5 s): PPO_MS forward time, PPO_PGZ gravity z (standing ≈ −1), PPO_VX command, PPO_FAIL state.",
    "Log DataFlash MDK / MDKQ / MDKV / MDKA: ogni tick con le 61 osservazioni e le 14 azioni → tools/log_parity.py le rigioca nell'ONNX (parità 2.5e-7).":
        "DataFlash log MDK / MDKQ / MDKV / MDKA: every tick with the 61 observations and 14 actions → tools/log_parity.py replays them through the ONNX (parity 2.5e-7).",
    "5. Percorso degli attuatori: dall'azione ai servo": "5. Actuator path: from the action to the servos",
    "La rete produce offset in radianti; ArduPilot li trasporta come PWM; la pianta li riconverte in target di posizione.": "The network outputs offsets in radians; ArduPilot carries them as PWM; the plant converts them back to position targets.",
    "Rete": "Network", "q0 = posa STAND2": "q0 = STAND2 pose", "funzioni 94..107 (Scripting1..14)": "functions 94..107 (Scripting1..14)",
    "Pianta / robot": "Plant / robot", "SITL: pacchetto servo udp:9002 → MuJoCo BAM": "SITL: servo packet udp:9002 → MuJoCo BAM", "HW: bus Dynamixel (tick)": "HW: Dynamixel bus (ticks)",
    "Perché Scripting1..14: sono funzioni servo già esistenti che ArduPilot non tocca (nessun mixer, nessun ESC), scrivibili con SRV_Channels::set_output_pwm senza modificare SRV_Channel":
        "Why Scripting1..14: existing servo functions ArduPilot never touches (no mixer, no ESC), writable with SRV_Channels::set_output_pwm without modifying SRV_Channel",
    "Risoluzione 3 mrad/µs: sufficiente per gli XL330 (1.5 mrad/tick); su hardware si passa ai tick Robotis e la conversione sparisce":
        "Resolution 3 mrad/µs: enough for the XL330 (1.5 mrad/tick); on hardware Robotis ticks are used and the conversion disappears",
    "PWM 0 = 'non guido' (disarmato): la pianta tiene il duck fermo in piedi; su hardware corrisponde a torque off":
        "PWM 0 = 'not driving' (disarmed): the plant keeps the duck pinned upright; on hardware this is torque off",
    "L'azione che finisce nei servo è la stessa salvata nella storia: la rete al passo dopo vede ciò che è stato davvero applicato":
        "The action sent to the servos is the one stored in the history: at the next step the network sees what was actually applied",
    "Feedback: la pianta restituisce q e q̇ reali dei giunti; il task non usa mai i target come surrogato": "Feedback: the plant returns the real joint q and q̇; the task never uses the targets as a surrogate",
    "6. L'integrazione in ArduRover, file per file": "6. The ArduRover integration, file by file",
    "Fork virtualrobotix/ardupilot, branch microduck-ppo, sopra master upstream del 24/09/2026. 7 file toccati + 1 libreria nuova.": "Fork virtualrobotix/ardupilot, branch microduck-ppo, on upstream master of 24 Sep 2026. 7 files touched + 1 new library.",
    "File": "File", "Modifica": "Change", "Perché": "Why",
    "libraries/AP_MicroDuck/  (nuova)": "libraries/AP_MicroDuck/  (new)",
    "il task: obs, filtro gravità, storia, stick, servo, MDK_*, log, telemetria; rete in C": "the task: obs, gravity filter, history, sticks, servos, MDK_*, log, telemetry; network in C",
    "membro g2.microduck; AP_SUBGROUPINFO \"MDK_\" indice 63": "member g2.microduck; AP_SUBGROUPINFO \"MDK_\" index 63",
    "parametri MDK_* visibili da GCS e salvati in EEPROM": "MDK_* parameters visible from the GCS and stored in EEPROM",
    "il task entra nello scheduler come qualunque libreria": "the task enters the scheduler like any other library",
    "link della libreria nel binario ardurover": "links the library into the ardurover binary",
    "tipo DATA_FLOAT_ARRAY14; chiavi joints/jpos, joints/jvel; copia in sitl->state": "type DATA_FLOAT_ARRAY14; keys joints/jpos, joints/jvel; copied into sitl->state",
    "il backend fisico consegna i giunti come consegna l'IMU": "the physics backend delivers the joints the same way it delivers the IMU",
    "sorgente SITL del joint feedback; su HW la sostituisce un driver Dynamixel": "SITL source of the joint feedback; on HW a Dynamixel driver replaces it",
    "Nessuna modifica a RC_Channels, SRV_Channel, AP_InertialSensor, AP_Arming, GCS_MAVLink: il task usa le loro API pubbliche. AP_MICRODUCK_ENABLED è 1 su SITL; le board devono abilitarlo (pesi ~770 KB + ~500 KB di flash in float32).":
        "No change to RC_Channels, SRV_Channel, AP_InertialSensor, AP_Arming, GCS_MAVLink: the task uses their public APIs. AP_MICRODUCK_ENABLED is 1 on SITL; boards must opt in (weights ~770 KB + ~500 KB of flash as float32).",
    "Parametri SITL (sitl/microduck.parm): SERVO1..14_FUNCTION 94..107, SERVOn_MIN/MAX 800/2200, SIM_RATE_HZ 200, SCHED_LOOP_RATE 200, INS_GYRO_FILTER 0, ARMING_CHECK 0, MDK_ENABLE 1.":
        "SITL parameters (sitl/microduck.parm): SERVO1..14_FUNCTION 94..107, SERVOn_MIN/MAX 800/2200, SIM_RATE_HZ 200, SCHED_LOOP_RATE 200, INS_GYRO_FILTER 0, ARMING_CHECK 0, MDK_ENABLE 1.",
    "7. Tempi e sincronizzazione": "7. Timing and synchronisation",
    "Una sola linea del tempo, guidata dal lock-step JSON.": "A single timeline, driven by the JSON lock-step.",
    "Loop ArduRover  (5 ms)": "ArduRover loop  (5 ms)", "MAVProxy  (asincrono)": "MAVProxy  (asynchronous)",
    "SITL manda i PWM → MuJoCo fa 1 passo (5 ms) → risponde col JSON → il SITL avanza il suo clock di 5 ms. Nessuno dei due può correre avanti.":
        "The SITL sends the PWM → MuJoCo takes 1 step (5 ms) → replies with the JSON → the SITL advances its clock by 5 ms. Neither side can run ahead.",
    "Il task a 50 Hz vede 4 passi fisici per azione, come in training (decimation 4). Il forward dura 0.25–0.37 ms sul Mac; su STM32H7 stimati 1–2 ms.":
        "The 50 Hz task sees 4 physics steps per action, as in training (decimation 4). The forward pass takes 0.25–0.37 ms on the Mac; 1–2 ms estimated on an STM32H7.",
    "8. Da .pt a C: come la rete addestrata entra nel firmware": "8. From .pt to C: how the trained network enters the firmware",
    "Il normalizzatore delle osservazioni è parte della rete, non del simulatore.": "The observation normalizer is part of the network, not of the simulator.",
    "Gemm/ELU… o Cartan": "Gemm/ELU… or Cartan", "mean[61], std[61], pesi, q0": "mean[61], std[61], weights, q0", "identico SITL / MCU": "identical SITL / MCU",
    "Verifica in tre punti": "Verified at three points",
    "tools/parity_check.py: 2201 osservazioni → ONNX Runtime vs binario C: max 3.8e-6 (MLP), 6.0e-5 (Cartan)": "tools/parity_check.py: 2201 observations → ONNX Runtime vs the C binary: max 3.8e-6 (MLP), 6.0e-5 (Cartan)",
    "tools/log_parity.py: le osservazioni loggate dal firmware in volo → ONNX vs le azioni che il firmware ha inviato: max 2.5e-7": "tools/log_parity.py: observations logged by the firmware while running → ONNX vs the actions the firmware sent: max 2.5e-7",
    "tools/policy_rollout_plant.py: la stessa policy sulla stessa pianta senza ArduPilot — separa problemi di pianta da problemi di firmware": "tools/policy_rollout_plant.py: the same policy on the same plant without ArduPilot — separates plant issues from firmware issues",
    "Le due reti (MDK_POLICY)": "The two networks (MDK_POLICY)",
    "0 = MLP 61→512→256→128→14 ELU, 197 896 parametri, 0.37 ms": "0 = MLP 61→512→256→128→14 ELU, 197 896 parameters, 0.37 ms",
    "1 = Cartan+DiLU 61→192→3×CartanLinear→14, 127 054 parametri (−36 %), 0.25 ms, operatore con exp/log in C": "1 = Cartan+DiLU 61→192→3×CartanLinear→14, 127 054 parameters (−36 %), 0.25 ms, exp/log operator in C",
    "9. Cosa cambia passando all'hardware": "9. What changes on hardware",
    "Solo il terzo processo: la pianta MuJoCo diventa il robot. AP_MicroDuck non cambia.": "Only the third process: the MuJoCo plant becomes the robot. AP_MicroDuck does not change.",
    "Blocco": "Block", "In SITL (oggi)": "In SITL (today)", "Su robot": "On the robot",
    "MuJoCo → JSON → INS simulata": "MuJoCo → JSON → simulated INS", "IMU del flight controller (stesso AP_InertialSensor)": "flight-controller IMU (same AP_InertialSensor)",
    "Feedback giunti": "Joint feedback", "driver bus Dynamixel (present position/velocity) → set_joint_feedback()": "Dynamixel bus driver (present position/velocity) → set_joint_feedback()",
    "Uscite servo": "Servo outputs", "SRV_Channels → pacchetto JSON PWM → MuJoCo BAM": "SRV_Channels → JSON PWM packet → MuJoCo BAM", "SRV_Channels → protocollo Robotis (tick) sugli XL330": "SRV_Channels → Robotis protocol (ticks) on the XL330",
    "Comandi": "Commands", "RC reale + GCS, identici": "real RC + GCS, identical",
    "policy_*.h in C, 0.3 ms su Mac": "policy_*.h in C, 0.3 ms on the Mac", "stessa sorgente; STM32H7: 1–2 ms stimati (MLP), Cartan da misurare": "same source; STM32H7: 1–2 ms estimated (MLP), Cartan to be measured",
    "Filtri IMU": "IMU filters", "stessa regola: nessun LPF lento sul gyro che alimenta la policy": "same rule: no slow LPF on the gyro that feeds the policy",
    "Il vincolo vero è il budget del microcontrollore: ~2×10⁵ MAC per passo (MLP) a 50 Hz più EKF, bus e logging. Il prossimo passo è lo stesso update() compilato su una Nucleo H7 con 1000 forward e misura p50/p99.":
        "The real constraint is the microcontroller budget: ~2×10⁵ MAC per step (MLP) at 50 Hz on top of EKF, buses and logging. Next step: the same update() compiled on a Nucleo H7, 1000 forward passes, p50/p99 measured.",
    "10. La demo e i risultati": "10. The demo and the results",
    "scripts/demo_mavproxy.py: un comando avvia pianta, SITL e MAVProxy e digita la sequenza dell'operatore.": "scripts/demo_mavproxy.py: one command starts plant, SITL and MAVProxy and types the operator's sequence.",
    "Sequenza digitata in MAVProxy": "Sequence typed into MAVProxy",
    "mode manual · rc all 1500 · arm throttle → 6 s in piedi": "mode manual · rc all 1500 · arm throttle → 6 s standing",
    "rc 2 2000 → avanti 3 s · rc 2 1000 → indietro 2 s": "rc 2 2000 → forward 3 s · rc 2 1000 → backward 2 s",
    "rc 1 1800 → laterale 2 s · rc 4 2000 → rotazione 90° (integrale di ATTITUDE.yawspeed) · rc 2 2000 → avanti 5 s": "rc 1 1800 → lateral 2 s · rc 4 2000 → 90° turn (integral of ATTITUDE.yawspeed) · rc 2 2000 → forward 5 s",
    "Nel video (docs/media/demo_mavproxy_mlp.mp4)": "In the video (docs/media/demo_mavproxy_mlp.mp4)",
    "box crediti e pannello a sinistra: comando MAVProxy e messaggio MAVLink corrispondente": "credit box and panel on the left: MAVProxy command and the matching MAVLink message",
    "badge a destra: fase e twist comandato in SI": "badge on the right: phase and commanded twist in SI units",
    "barra in basso: tempo, posizione, tilt e yaw del robot in MuJoCo": "bottom bar: time, position, tilt and yaw of the robot in MuJoCo",
    "Parità C vs ONNX": "Parity C vs ONNX", "Parità in-situ (log firmware)": "In-situ parity (firmware log)", "Forward nel SITL": "Forward pass in SITL",
    "Flash pesi (float32)": "Weights in flash (float32)", "Batteria HIL 8 passi": "HIL battery, 8 steps", "Avanti 0.3/0.4 m/s 15 s": "Forward 0.3/0.4 m/s, 15 s", "nessuna caduta": "no fall",
    "Il tracking di velocità (~0.2 m/s a comando 0.4) è lo stesso del rollout Python sulla stessa pianta CPU: è il gap MuJoCo-CPU vs MuJoCo-Warp del training, non l'integrazione ArduPilot.":
        "Speed tracking (~0.2 m/s at a 0.4 command) is the same as the Python rollout on the same CPU plant: it is the MuJoCo-CPU vs MuJoCo-Warp gap of training, not the ArduPilot integration.",
}



def text(slide, x, y, w, h, lines, size=14, bold=False, color=None, align=PP_ALIGN.LEFT, font=None, anchor=None):
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    if anchor is not None:
        tf.vertical_anchor = anchor
    if isinstance(lines, str):
        lines = [lines]
    for i, ln in enumerate(lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        if isinstance(ln, tuple):
            ln, opts = ln
        else:
            opts = {}
        r = p.add_run()
        r.text = tr(ln)
        r.font.size = Pt(opts.get("size", size))
        r.font.bold = opts.get("bold", bold)
        c = opts.get("color", color)
        if c is not None:
            r.font.color.rgb = c
        if font or opts.get("font"):
            r.font.name = opts.get("font", font)
        if opts.get("space"):
            p.space_before = Pt(opts["space"])
    return tb


def title(slide, t, sub=None):
    global _n
    _n += 1
    text(slide, Inches(0.5), Inches(0.25), Inches(12.3), Inches(0.8), t, size=28, bold=True, color=NAVY)
    ln = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(0.5), Inches(1.05), Inches(12.83), Inches(1.05))
    ln.line.color.rgb = RED
    ln.line.width = Pt(1.5)
    if sub:
        text(slide, Inches(0.5), Inches(1.1), Inches(12.3), Inches(0.5), sub, size=14, color=GREY)
    text(slide, Inches(0.5), Inches(7.05), Inches(10), Inches(0.35), CREDIT, size=9, color=GREY)
    text(slide, Inches(12.1), Inches(7.05), Inches(0.8), Inches(0.35), str(_n), size=9, color=GREY, align=PP_ALIGN.RIGHT)


def box(slide, x, y, w, h, lines, fill=LIGHT, line=NAVY, size=12, bold_first=True, color=NAVY, shape=MSO_SHAPE.ROUNDED_RECTANGLE):
    s = slide.shapes.add_shape(shape, x, y, w, h)
    s.fill.solid()
    s.fill.fore_color.rgb = fill
    s.line.color.rgb = line
    s.line.width = Pt(1.25)
    tf = s.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.08)
    tf.margin_top = tf.margin_bottom = Inches(0.04)
    if isinstance(lines, str):
        lines = [lines]
    for i, ln in enumerate(lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = PP_ALIGN.CENTER
        r = p.add_run()
        r.text = tr(ln)
        r.font.size = Pt(size if i else size + 1)
        r.font.bold = bold_first and i == 0
        r.font.color.rgb = color
    return s


def arrow(slide, x1, y1, x2, y2, color=GREY, width=1.5, dashed=False, label=None, label_dy=-0.3, label_size=10):
    c = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, x1, y1, x2, y2)
    c.line.color.rgb = color
    c.line.width = Pt(width)
    ln = c.line._get_or_add_ln()
    ln.append(ln.makeelement(qn("a:tailEnd"), {"type": "triangle"}))
    if dashed:
        ln.append(ln.makeelement(qn("a:prstDash"), {"val": "dash"}))
    if label:
        text(slide, min(x1, x2), min(y1, y2) + Inches(label_dy), abs(x2 - x1) or Inches(2.6), Inches(0.35), label,
             size=label_size, color=color, align=PP_ALIGN.CENTER)
    return c


def frame(slide, x, y, w, h, label, color=NAVY):
    s = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, x, y, w, h)
    s.fill.background()
    s.line.color.rgb = color
    s.line.width = Pt(1.5)
    s.line.dash_style = 4
    tf = s.text_frame
    tf.vertical_anchor = MSO_ANCHOR.TOP
    tf.margin_top = Inches(0.06)
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.LEFT
    r = p.add_run()
    r.text = label
    r.font.size = Pt(13)
    r.font.bold = True
    r.font.color.rgb = color
    return s


def bullets(slide, x, y, w, h, items, size=15):
    lines = []
    for it in items:
        if isinstance(it, tuple):
            lines.append((tr(it[0]), {"bold": True, "color": NAVY, "size": size + 2, "space": 8}))
            for sub in it[1]:
                lines.append(("•  " + tr(sub), {"size": size, "space": 3}))
        else:
            lines.append(("•  " + tr(it), {"size": size, "space": 4}))
    text(slide, x, y, w, h, lines)



def build(lang: str, out: Path) -> None:
    global prs, BLANK, _n, LANG
    LANG = lang
    _n = 0
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)
    BLANK = prs.slide_layouts[6]
    OUT = out
    # ------------------------------------------------------------------ 1 cover
    s = prs.slides.add_slide(BLANK)
    _n += 1
    text(s, Inches(0.8), Inches(1.5), Inches(11.7), Inches(1.4),
         [("MicroDuck su ArduPilot", {"size": 40, "bold": True, "color": NAVY}),
          ("Come sono collegati MAVProxy, ArduRover, la rete PPO e MuJoCo", {"size": 22, "color": RED})],
         align=PP_ALIGN.CENTER)
    text(s, Inches(0.8), Inches(3.6), Inches(11.7), Inches(1.6),
         [("Roberto Navoni — DelphyAI LAB · r.navoni74@gmail.com · 24 settembre 2026", {"size": 14, "color": GREY}),
          ("Repo: github.com/virtualrobotix/microduck-ap-ppo-sitl  ·  fork ArduPilot: virtualrobotix/ardupilot @ microduck-ppo", {"size": 13, "color": GREY})],
         align=PP_ALIGN.CENTER)
    text(s, Inches(1.2), Inches(5.2), Inches(10.9), Inches(1.2),
         "Un solo firmware ArduRover. La rete neurale (61 osservazioni → 14 servo, 50 Hz) è un task dello scheduler. "
         "MuJoCo è il corpo del robot, collegato al SITL come un motore fisico esterno. Il comando arriva da MAVProxy via MAVLink.",
         size=15, color=NAVY, align=PP_ALIGN.CENTER)
    text(s, Inches(0.5), Inches(7.05), Inches(10), Inches(0.35), CREDIT, size=9, color=GREY)

    # ------------------------------------------------------------------ 2 the three processes
    s = prs.slides.add_slide(BLANK)
    title(s, "1. Tre processi, due collegamenti", "In simulazione: GCS, firmware, corpo del robot. Su hardware il terzo processo è il robot vero.")
    y = Inches(2.0)
    h = Inches(3.6)
    gcs = box(s, Inches(0.5), y, Inches(3.2), h, ["MAVProxy (GCS)", "", "operatore o script", "arm / disarm", "stick:  rc N <pwm>", "mode manual / hold", "param set MDK_*", "telemetria PPO_*"], fill=GREEN_FILL, line=GREEN, color=GREEN, size=13)
    ap = box(s, Inches(4.7), y, Inches(4.0), h, ["ArduRover SITL", "(un solo binario ardurover)", "", "RC_Channels → twist SI", "AP_InertialSensor → gyro, gravità", "AP_MicroDuck: rete PPO 50 Hz", "SRV_Channels: 14 PWM", "backend SIM_JSON"], fill=BLUE_FILL, line=NAVY, color=NAVY, size=13)
    mj = box(s, Inches(9.7), y, Inches(3.1), h, ["Pianta MuJoCo", "plant/mujoco_json_plant.py", "", "14 servo XL330 (modello BAM)", "fisica 200 Hz (5 ms)", "IMU, encoder giunti", "viewer / video"], fill=RED_FILL, line=RED, color=RED, size=13)
    arrow(s, Inches(3.7), y + Inches(1.3), Inches(4.7), y + Inches(1.3), color=GREEN, width=2.25, label="MAVLink  tcp:5760", label_dy=-0.38, label_size=12)
    arrow(s, Inches(4.7), y + Inches(2.3), Inches(3.7), y + Inches(2.3), color=GREEN, width=1.5, dashed=True, label="STATUSTEXT, NAMED_VALUE_FLOAT", label_dy=0.05, label_size=10)
    arrow(s, Inches(8.7), y + Inches(1.3), Inches(9.7), y + Inches(1.3), color=RED, width=2.25, label="udp:9002  16 PWM", label_dy=-0.38, label_size=12)
    arrow(s, Inches(9.7), y + Inches(2.3), Inches(8.7), y + Inches(2.3), color=RED, width=2.25, label="udp:9003  JSON", label_dy=0.05, label_size=12)
    text(s, Inches(0.5), Inches(5.85), Inches(12.3), Inches(1.1),
         [("Lock-step: il SITL invia i PWM e aspetta il JSON; MuJoCo fa un passo da 5 ms per frame (SIM_RATE_HZ 200). Il task PPO gira ogni 4 frame = 50 Hz, come in training.", {"size": 13}),
          ("Il task PPO non ha alcun collegamento con MuJoCo: vede solo API ArduPilot. Sostituire la pianta con IMU + bus Dynamixel reali non cambia una riga di AP_MicroDuck.", {"size": 13, "color": RED, "bold": True, "space": 6})])

    # ------------------------------------------------------------------ 3 inside ArduRover
    s = prs.slides.add_slide(BLANK)
    title(s, "2. Dentro ArduRover: il loop a 50 Hz di AP_MicroDuck", "Tutto ciò che la rete vede passa dalle librerie standard di ArduPilot.")
    fr = frame(s, Inches(0.5), Inches(1.7), Inches(9.3), Inches(4.9), "ArduRover — scheduler (SCHED_LOOP_RATE 200)")
    bw, bh = Inches(2.15), Inches(0.8)
    ys = [Inches(2.2), Inches(3.15), Inches(4.1), Inches(5.05)]
    ins = box(s, Inches(0.7), ys[0], bw, bh, ["AP_InertialSensor", "gyro, accel FRD"], size=11)
    jfb = box(s, Inches(0.7), ys[1], bw, bh, ["Joint feedback", "q, q̇ (14) da sitl->state"], size=11)
    rc = box(s, Inches(0.7), ys[2], bw, bh, ["RC_Channels", "stick 1/2/4 → vx vy ωz"], size=11)
    hist = box(s, Inches(0.7), ys[3], bw, bh, ["Storia", "azione precedente (14)"], size=11)
    cf = box(s, Inches(3.1), ys[0], Inches(2.0), bh, ["Filtro gravità", "IMU-only, no EKF"], size=11)
    obs = box(s, Inches(5.35), Inches(2.9), Inches(1.9), Inches(1.5), ["build_obs", "61 float32", "frame trunk FLU"], fill=BLUE_FILL, size=11)
    net = box(s, Inches(7.45), Inches(2.9), Inches(2.15), Inches(1.5), ["Forward C", "mean/std + rete", "MLP | Cartan", "≈0.3 ms"], fill=BLUE_FILL, size=11)
    for b in (jfb, rc, hist):
        arrow(s, b.left + b.width, b.top + b.height // 2, obs.left, obs.top + obs.height // 2)
    arrow(s, ins.left + ins.width, ins.top + ins.height // 2, cf.left, cf.top + cf.height // 2)
    arrow(s, cf.left + cf.width, cf.top + cf.height // 2, obs.left, obs.top + Inches(0.3))
    arrow(s, obs.left + obs.width, obs.top + obs.height // 2, net.left, net.top + net.height // 2)
    srv = box(s, Inches(7.45), Inches(5.05), Inches(2.15), bh, ["SRV_Channels", "Scripting1..14 → PWM"], size=11)
    arrow(s, net.left + net.width // 2, net.top + net.height, srv.left + srv.width // 2, srv.top, label="q_target = q0 + a", label_dy=0.1, label_size=10)
    text(s, Inches(3.1), Inches(5.55), Inches(4.0), Inches(0.5), "a applicata → storia (obs[34:48]) al tick successivo", size=10, color=GREY)
    bullets(s, Inches(10.0), Inches(1.7), Inches(3.0), Inches(5.0), [
        "update_attitude() a loop rate: propaga la direzione della gravità col gyro, la corregge con l'accelerometro (MDK_ATT_TAU 0.5 s)",
        "update() a 50 Hz: obs → forward → clip → PWM",
        "Failsafe: disarmato, feedback giunti assente o vecchio > MDK_WD_MS → azione 0 = posa stand (anche nella storia)",
        "HOLD (MDK_HOLD_MODE 4) → twist 0",
        "PWM: 1500 + q/0.003 (1 µs = 3 mrad), SERVOn 800..2200",
    ], size=12)

    # ------------------------------------------------------------------ 4 sensor path
    s = prs.slides.add_slide(BLANK)
    title(s, "3. Percorso dei sensori: da MuJoCo all'osservazione", "Frame diversi, unità diverse: la pianta parla ArduPilot (FRD/NED), il task parla training (FLU).")
    b1 = box(s, Inches(0.5), Inches(2.0), Inches(2.6), Inches(1.5), ["MuJoCo", "gyro/accel sito IMU (FLU)", "quaternione trunk (z-up)", "qpos, qvel 14 giunti"], fill=RED_FILL, line=RED, color=RED, size=11)
    b2 = box(s, Inches(3.6), Inches(2.0), Inches(2.6), Inches(1.5), ["JSON udp:9003", "imu.gyro, imu.accel_body (FRD)", "quaternion (NED→FRD)", "joints.jpos / jvel"], fill=LIGHT, size=11)
    b3 = box(s, Inches(6.7), Inches(2.0), Inches(2.9), Inches(1.5), ["SITL / SIM_JSON.cpp", "→ INS simulata (gyro, accel)", "→ sitl->state.joint_pos/vel", "(estensione di questo repo)"], fill=BLUE_FILL, size=11)
    b4 = box(s, Inches(10.1), Inches(2.0), Inches(2.7), Inches(1.5), ["AP_MicroDuck", "gyro FRD→FLU: (x, −y, −z)", "gravità: filtro → (x, −y, −z)", "q − q0, q̇"], fill=BLUE_FILL, size=11)
    for a, b in ((b1, b2), (b2, b3), (b3, b4)):
        arrow(s, a.left + a.width, a.top + a.height // 2, b.left, b.top + b.height // 2, width=2)
    bullets(s, Inches(0.5), Inches(3.9), Inches(12.3), Inches(3.0), [
        ("Conversione dei frame (fatta dalla pianta)", [
            "vettori: (x, y, z)_MuJoCo → (x, −y, −z)_ArduPilot  — vale per gyro, accel, posizione, velocità",
            "quaternione: (w, x, y, z) → (w, x, −y, −z)",
            "in piedi: accel_body = (0, 0, −9.81) come ArduPilot si aspetta; gravità proiettata nel task = (0, 0, −1) come in training"]),
        ("Lezione del primo test", [
            "ArduRover ha INS_GYRO_FILTER 4 Hz di default: il gyro arrivava alla rete con decine di ms di ritardo → caduta in 1 s",
            "INS_GYRO_FILTER 0 (o 40) → stabile. Nessun passa-basso lento sul gyro che alimenta la policy, anche su hardware"]),
    ], size=13)

    # ------------------------------------------------------------------ 5 command path
    s = prs.slides.add_slide(BLANK)
    title(s, "4. Percorso dei comandi: da MAVProxy al twist", "Ogni comando digitato in MAVProxy è un messaggio MAVLink standard; ArduPilot non è stato modificato qui.")
    rows = [
        ("Comando MAVProxy", "Messaggio MAVLink", "Cosa fa in ArduRover", "Effetto sulla rete"),
        ("arm throttle [force]", "COMMAND_LONG  MAV_CMD_COMPONENT_ARM_DISARM p1=1", "AP_Arming → soft armed", "PPO_FAIL 1→0, servo attivi, la policy tiene in piedi"),
        ("rc 2 2000", "RC_CHANNELS_OVERRIDE chan2=2000 (da sysid 255)", "RC_Channels: norm_input_dz = +1.0", "obs[48] vx = +1.0 × MDK_VX_MAX = 0.4 m/s"),
        ("rc 1 1800", "RC_CHANNELS_OVERRIDE chan1=1800", "norm = +0.6", "obs[49] vy = 0.6 × 0.3 = 0.18 m/s"),
        ("rc 4 2000", "RC_CHANNELS_OVERRIDE chan4=2000", "norm = +1.0", "obs[50] ωz = 1.0 rad/s"),
        ("mode hold", "COMMAND_LONG  MAV_CMD_DO_SET_MODE custom_mode=4", "Rover in HOLD", "MDK_HOLD_MODE → twist forzato a 0: sta fermo"),
        ("param set MDK_POLICY 1", "PARAM_SET", "AP_Param", "prossimo tick usa la rete Cartan"),
        ("disarm", "COMMAND_LONG  ARM_DISARM p1=0", "soft disarmed", "azione 0, storia azzerata, PWM 0 (pianta: idle)"),
    ]
    tbl = s.shapes.add_table(len(rows), 4, Inches(0.5), Inches(1.7), Inches(12.3), Inches(0.42) * len(rows)).table
    widths = [Inches(2.4), Inches(3.9), Inches(2.6), Inches(3.4)]
    for i, w in enumerate(widths):
        tbl.columns[i].width = w
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text = ""
            r = cell.text_frame.paragraphs[0].add_run()
            r.text = tr(val)
            r.font.size = Pt(11 if ri else 12)
            r.font.bold = ri == 0
            r.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF) if ri == 0 else RGBColor(0x20, 0x20, 0x20)
            cell.fill.solid()
            cell.fill.fore_color.rgb = NAVY if ri == 0 else (LIGHT if ri % 2 == 0 else RGBColor(0xFF, 0xFF, 0xFF))
    text(s, Inches(0.5), Inches(5.6), Inches(12.3), Inches(1.2),
         [("Telemetria di ritorno (NAMED_VALUE_FLOAT ogni 0.5 s): PPO_MS tempo del forward, PPO_PGZ gravità z (in piedi ≈ −1), PPO_VX comando, PPO_FAIL stato.", {"size": 13}),
          ("Log DataFlash MDK / MDKQ / MDKV / MDKA: ogni tick con le 61 osservazioni e le 14 azioni → tools/log_parity.py le rigioca nell'ONNX (parità 2.5e-7).", {"size": 13, "space": 6})])

    # ------------------------------------------------------------------ 6 actuator path
    s = prs.slides.add_slide(BLANK)
    title(s, "5. Percorso degli attuatori: dall'azione ai servo", "La rete produce offset in radianti; ArduPilot li trasporta come PWM; la pianta li riconverte in target di posizione.")
    c1 = box(s, Inches(0.5), Inches(2.0), Inches(2.5), Inches(1.5), ["Rete", "a[14] rad", "clip ±MDK_ACT_MAX"], fill=BLUE_FILL, size=11)
    c2 = box(s, Inches(3.4), Inches(2.0), Inches(2.5), Inches(1.5), ["q_target", "= q0 + a", "q0 = posa STAND2"], fill=BLUE_FILL, size=11)
    c3 = box(s, Inches(6.3), Inches(2.0), Inches(2.8), Inches(1.5), ["SRV_Channels", "pwm = 1500 + q/0.003", "funzioni 94..107 (Scripting1..14)"], fill=BLUE_FILL, size=11)
    c4 = box(s, Inches(9.5), Inches(2.0), Inches(3.3), Inches(1.5), ["Pianta / robot", "SITL: pacchetto servo udp:9002 → MuJoCo BAM", "HW: bus Dynamixel (tick)"], fill=RED_FILL, line=RED, color=RED, size=11)
    for a, b in ((c1, c2), (c2, c3), (c3, c4)):
        arrow(s, a.left + a.width, a.top + a.height // 2, b.left, b.top + b.height // 2, width=2)
    bullets(s, Inches(0.5), Inches(3.9), Inches(12.3), Inches(3.0), [
        "Perché Scripting1..14: sono funzioni servo già esistenti che ArduPilot non tocca (nessun mixer, nessun ESC), scrivibili con SRV_Channels::set_output_pwm senza modificare SRV_Channel",
        "Risoluzione 3 mrad/µs: sufficiente per gli XL330 (1.5 mrad/tick); su hardware si passa ai tick Robotis e la conversione sparisce",
        "PWM 0 = 'non guido' (disarmato): la pianta tiene il duck fermo in piedi; su hardware corrisponde a torque off",
        "L'azione che finisce nei servo è la stessa salvata nella storia: la rete al passo dopo vede ciò che è stato davvero applicato",
        "Feedback: la pianta restituisce q e q̇ reali dei giunti; il task non usa mai i target come surrogato",
    ], size=13)

    # ------------------------------------------------------------------ 7 the ArduRover integration in detail
    s = prs.slides.add_slide(BLANK)
    title(s, "6. L'integrazione in ArduRover, file per file", "Fork virtualrobotix/ardupilot, branch microduck-ppo, sopra master upstream del 24/09/2026. 7 file toccati + 1 libreria nuova.")
    rows = [
        ("File", "Modifica", "Perché"),
        ("libraries/AP_MicroDuck/  (nuova)", "AP_MicroDuck.{h,cpp}, microduck_infer.{h,c}, policy_mlp.h, policy_cartan.h, AP_MicroDuck_config.h", "il task: obs, filtro gravità, storia, stick, servo, MDK_*, log, telemetria; rete in C"),
        ("Rover/Parameters.h / .cpp", "membro g2.microduck; AP_SUBGROUPINFO \"MDK_\" indice 63", "parametri MDK_* visibili da GCS e salvati in EEPROM"),
        ("Rover/Rover.cpp", "SCHED_TASK_CLASS update_attitude 400 Hz (→ loop rate), update 50 Hz", "il task entra nello scheduler come qualunque libreria"),
        ("Rover/wscript", "+ 'AP_MicroDuck'", "link della libreria nel binario ardurover"),
        ("libraries/SITL/SIM_JSON.h / .cpp", "tipo DATA_FLOAT_ARRAY14; chiavi joints/jpos, joints/jvel; copia in sitl->state", "il backend fisico consegna i giunti come consegna l'IMU"),
        ("libraries/SITL/SITL.h", "sitl_fdm: joint_pos[16], joint_vel[16], joint_count, joint_time_us", "sorgente SITL del joint feedback; su HW la sostituisce un driver Dynamixel"),
    ]
    tbl = s.shapes.add_table(len(rows), 3, Inches(0.5), Inches(1.7), Inches(12.3), Inches(0.55) * len(rows)).table
    for i, w in enumerate([Inches(3.2), Inches(5.0), Inches(4.1)]):
        tbl.columns[i].width = w
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text = ""
            r = cell.text_frame.paragraphs[0].add_run()
            r.text = tr(val)
            r.font.size = Pt(11 if ri else 12)
            r.font.bold = ri == 0 or ci == 0
            r.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF) if ri == 0 else RGBColor(0x20, 0x20, 0x20)
            cell.fill.solid()
            cell.fill.fore_color.rgb = NAVY if ri == 0 else (LIGHT if ri % 2 == 0 else RGBColor(0xFF, 0xFF, 0xFF))
    text(s, Inches(0.5), Inches(5.75), Inches(12.3), Inches(1.2),
         [("Nessuna modifica a RC_Channels, SRV_Channel, AP_InertialSensor, AP_Arming, GCS_MAVLink: il task usa le loro API pubbliche. AP_MICRODUCK_ENABLED è 1 su SITL; le board devono abilitarlo (pesi ~770 KB + ~500 KB di flash in float32).", {"size": 13}),
          ("Parametri SITL (sitl/microduck.parm): SERVO1..14_FUNCTION 94..107, SERVOn_MIN/MAX 800/2200, SIM_RATE_HZ 200, SCHED_LOOP_RATE 200, INS_GYRO_FILTER 0, ARMING_CHECK 0, MDK_ENABLE 1.", {"size": 13, "space": 6})])

    # ------------------------------------------------------------------ 8 timing
    s = prs.slides.add_slide(BLANK)
    title(s, "7. Tempi e sincronizzazione", "Una sola linea del tempo, guidata dal lock-step JSON.")
    y0 = Inches(2.0)
    lanes = [("MuJoCo  (5 ms)", RED, 40), ("SITL frame / INS  (5 ms)", NAVY, 40), ("Loop ArduRover  (5 ms)", NAVY, 40), ("AP_MicroDuck::update  (20 ms)", GREEN, 10), ("MAVProxy  (asincrono)", ORANGE, 0)]
    x0, xw = Inches(3.4), Inches(9.2)
    for i, (name, col, n) in enumerate(lanes):
        y = y0 + Inches(0.75) * i
        text(s, Inches(0.5), y, Inches(2.8), Inches(0.5), name, size=12, bold=True, color=col)
        ln = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, x0, y + Inches(0.25), x0 + xw, y + Inches(0.25))
        ln.line.color.rgb = GREY
        ln.line.width = Pt(0.75)
        for k in range(n):
            xx = x0 + xw * k / n
            m = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, xx, y + Inches(0.1), Inches(0.05) if n > 10 else Inches(0.18), Inches(0.3))
            m.fill.solid()
            m.fill.fore_color.rgb = col
            m.line.fill.background()
        if n == 0:
            for xx_in in (0.8, 3.9, 6.7):
                m = s.shapes.add_shape(MSO_SHAPE.OVAL, x0 + Inches(xx_in), y + Inches(0.1), Inches(0.3), Inches(0.3))
                m.fill.solid()
                m.fill.fore_color.rgb = col
                m.line.fill.background()
    text(s, Inches(3.4), y0 + Inches(3.85), Inches(9.2), Inches(0.4), "200 ms", size=11, color=GREY, align=PP_ALIGN.CENTER)
    bullets(s, Inches(0.5), Inches(6.0), Inches(12.3), Inches(1.0), [
        "SITL manda i PWM → MuJoCo fa 1 passo (5 ms) → risponde col JSON → il SITL avanza il suo clock di 5 ms. Nessuno dei due può correre avanti.",
        "Il task a 50 Hz vede 4 passi fisici per azione, come in training (decimation 4). Il forward dura 0.25–0.37 ms sul Mac; su STM32H7 stimati 1–2 ms.",
    ], size=12)

    # ------------------------------------------------------------------ 9 network path
    s = prs.slides.add_slide(BLANK)
    title(s, "8. Da .pt a C: come la rete addestrata entra nel firmware", "Il normalizzatore delle osservazioni è parte della rete, non del simulatore.")
    n1 = box(s, Inches(0.5), Inches(2.1), Inches(2.3), Inches(1.6), ["model_1999.pt", "PyTorch: actor, critic,", "optimizer, normalizer", "(mjlab + rsl_rl, 2048×2000)"], fill=LIGHT, size=11)
    n2 = box(s, Inches(3.2), Inches(2.1), Inches(2.3), Inches(1.6), [".onnx", "actor(normalizer(obs))", "Sub(mean) → Div(std) →", "Gemm/ELU… o Cartan"], fill=LIGHT, size=11)
    n3 = box(s, Inches(5.9), Inches(2.1), Inches(2.5), Inches(1.6), ["policy_*.h", "static const float", "mean[61], std[61], pesi, q0", "773 KB MLP / 496 KB Cartan"], fill=BLUE_FILL, size=11)
    n4 = box(s, Inches(8.8), Inches(2.1), Inches(2.3), Inches(1.6), ["microduck_infer.c", "forward float32", "expf/logf, no heap", "identico SITL / MCU"], fill=BLUE_FILL, size=11)
    n5 = box(s, Inches(11.4), Inches(2.1), Inches(1.4), Inches(1.6), ["AP_MicroDuck", "update()", "50 Hz"], fill=BLUE_FILL, size=11)
    for a, b, lab in ((n1, n2, "scripts/export.py"), (n2, n3, "tools/export_*_c.py"), (n3, n4, "#include"), (n4, n5, "")):
        arrow(s, a.left + a.width, a.top + a.height // 2, b.left, b.top + b.height // 2, width=2, label=lab or None, label_dy=-0.35, label_size=9)
    bullets(s, Inches(0.5), Inches(4.1), Inches(12.3), Inches(2.8), [
        ("Verifica in tre punti", [
            "tools/parity_check.py: 2201 osservazioni → ONNX Runtime vs binario C: max 3.8e-6 (MLP), 6.0e-5 (Cartan)",
            "tools/log_parity.py: le osservazioni loggate dal firmware in volo → ONNX vs le azioni che il firmware ha inviato: max 2.5e-7",
            "tools/policy_rollout_plant.py: la stessa policy sulla stessa pianta senza ArduPilot — separa problemi di pianta da problemi di firmware"]),
        ("Le due reti (MDK_POLICY)", [
            "0 = MLP 61→512→256→128→14 ELU, 197 896 parametri, 0.37 ms",
            "1 = Cartan+DiLU 61→192→3×CartanLinear→14, 127 054 parametri (−36 %), 0.25 ms, operatore con exp/log in C"]),
    ], size=12)

    # ------------------------------------------------------------------ 10 hardware
    s = prs.slides.add_slide(BLANK)
    title(s, "9. Cosa cambia passando all'hardware", "Solo il terzo processo: la pianta MuJoCo diventa il robot. AP_MicroDuck non cambia.")
    rows = [
        ("Blocco", "In SITL (oggi)", "Su robot"),
        ("IMU", "MuJoCo → JSON → INS simulata", "IMU del flight controller (stesso AP_InertialSensor)"),
        ("Feedback giunti", "JSON joints → sitl->state", "driver bus Dynamixel (present position/velocity) → set_joint_feedback()"),
        ("Uscite servo", "SRV_Channels → pacchetto JSON PWM → MuJoCo BAM", "SRV_Channels → protocollo Robotis (tick) sugli XL330"),
        ("Comandi", "MAVProxy rc / mode / arm", "RC reale + GCS, identici"),
        ("Rete", "policy_*.h in C, 0.3 ms su Mac", "stessa sorgente; STM32H7: 1–2 ms stimati (MLP), Cartan da misurare"),
        ("Filtri IMU", "INS_GYRO_FILTER 0", "stessa regola: nessun LPF lento sul gyro che alimenta la policy"),
    ]
    tbl = s.shapes.add_table(len(rows), 3, Inches(0.5), Inches(1.7), Inches(12.3), Inches(0.5) * len(rows)).table
    for i, w in enumerate([Inches(2.4), Inches(4.6), Inches(5.3)]):
        tbl.columns[i].width = w
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text = ""
            r = cell.text_frame.paragraphs[0].add_run()
            r.text = tr(val)
            r.font.size = Pt(12 if ri else 13)
            r.font.bold = ri == 0 or ci == 0
            r.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF) if ri == 0 else RGBColor(0x20, 0x20, 0x20)
            cell.fill.solid()
            cell.fill.fore_color.rgb = NAVY if ri == 0 else (LIGHT if ri % 2 == 0 else RGBColor(0xFF, 0xFF, 0xFF))
    text(s, Inches(0.5), Inches(5.6), Inches(12.3), Inches(1.2),
         [("Il vincolo vero è il budget del microcontrollore: ~2×10⁵ MAC per passo (MLP) a 50 Hz più EKF, bus e logging. Il prossimo passo è lo stesso update() compilato su una Nucleo H7 con 1000 forward e misura p50/p99.", {"size": 13})])

    # ------------------------------------------------------------------ 11 demo & results
    s = prs.slides.add_slide(BLANK)
    title(s, "10. La demo e i risultati", "scripts/demo_mavproxy.py: un comando avvia pianta, SITL e MAVProxy e digita la sequenza dell'operatore.")
    bullets(s, Inches(0.5), Inches(1.6), Inches(6.2), Inches(5.3), [
        ("Sequenza digitata in MAVProxy", [
            "param set MDK_POLICY 0|1 · param set MDK_ENABLE 1",
            "mode manual · rc all 1500 · arm throttle → 6 s in piedi",
            "rc 2 2000 → avanti 3 s · rc 2 1000 → indietro 2 s",
            "rc 1 1800 → laterale 2 s · rc 4 2000 → rotazione 90° (integrale di ATTITUDE.yawspeed) · rc 2 2000 → avanti 5 s",
            "mode hold → twist 0 · mode manual · disarm"]),
        ("Nel video (docs/media/demo_mavproxy_mlp.mp4)", [
            "box crediti e pannello a sinistra: comando MAVProxy e messaggio MAVLink corrispondente",
            "badge a destra: fase e twist comandato in SI",
            "barra in basso: tempo, posizione, tilt del robot in MuJoCo"]),
    ], size=12)
    rows = [
        ("", "MLP", "Cartan"),
        ("Parità C vs ONNX", "3.8e-6", "6.0e-5"),
        ("Parità in-situ (log firmware)", "2.5e-7", "1.3e-6 (p99)"),
        ("Forward nel SITL", "0.37 ms", "0.25 ms"),
        ("Flash pesi (float32)", "773 KB", "496 KB"),
        ("Batteria HIL 8 passi", "8/8 PASS", "8/8 PASS"),
        ("Avanti 0.3/0.4 m/s 15 s", "nessuna caduta", "—"),
    ]
    tbl = s.shapes.add_table(len(rows), 3, Inches(7.0), Inches(1.7), Inches(5.8), Inches(0.45) * len(rows)).table
    for i, w in enumerate([Inches(2.8), Inches(1.5), Inches(1.5)]):
        tbl.columns[i].width = w
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text = ""
            r = cell.text_frame.paragraphs[0].add_run()
            r.text = tr(val)
            r.font.size = Pt(12)
            r.font.bold = ri == 0
            r.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF) if ri == 0 else RGBColor(0x20, 0x20, 0x20)
            cell.fill.solid()
            cell.fill.fore_color.rgb = NAVY if ri == 0 else (LIGHT if ri % 2 == 0 else RGBColor(0xFF, 0xFF, 0xFF))
    text(s, Inches(7.0), Inches(5.1), Inches(5.8), Inches(1.6),
         [("Il tracking di velocità (~0.2 m/s a comando 0.4) è lo stesso del rollout Python sulla stessa pianta CPU: è il gap MuJoCo-CPU vs MuJoCo-Warp del training, non l'integrazione ArduPilot.", {"size": 12, "color": GREY})])

    prs.save(str(OUT))
    print(f"wrote {OUT} ({_n} slides)")


if __name__ == "__main__":
    build("it", ROOT / "docs" / "architettura-integrazione.pptx")
    build("en", ROOT / "docs" / "architecture-integration.pptx")
