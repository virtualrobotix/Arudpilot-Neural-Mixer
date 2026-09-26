# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Single source of truth for the robot catalog.

tools/robots/build_catalog.py writes, for every robot here:
  robots/<id>/robot/profile.json   topology + upstream + simulation block
  robots/<id>/robot/ppo.yaml       PPO architecture and ArduPilot-compatible env
  docs/robots/<id>.md              robot page
  docs/robots/README.md            configuration table

Upstream facts (paths, joints, servos, rates, licenses) were read from the
upstream repositories in September 2026; the "source" notes say where.
"""

from __future__ import annotations

import math

DEG = math.pi / 180.0

# Hidden sizes shared by every robot: the MicroDuck network already validated on a Pixhawk 6C.
MLP_HIDDEN = [512, 256, 128]

ROBOTS: dict[str, dict] = {
    "microduck": {
        "index": 0,
        "class": "biped",
        "display_name": "MicroDuck",
        "maker": "Pollen Robotics / Hugging Face",
        "status": "policy",
        "joint_names": [
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
            "neck_pitch", "head_pitch", "head_yaw", "head_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
        ],
        "q0": [0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0,
               0.0, 0.0873, 0.4579, 0.0049, -0.4530],
        "extra_cmd_dim": 10,           # head pose 4 + body pose 6, zero on the autopilot
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.4, 0.4], "vy": [-0.3, 0.3], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/pollen-robotics/microduck_rl",
            "license": "vedi upstream",
            "sim_model": "src/mjlab_microduck/robot/microduck/scene.xml",
            "cad": "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
            "bom": "https://huggingface.co/spaces/pollen-robotics/microduck-simulator",
            "training": "mjlab 1.3.0 + rsl_rl PPO (MuJoCo Warp)",
            "published_policy": "addestrata nel laboratorio NOESIS EXPERIMENT; ONNX in policies/ di questo repo",
        },
        "sim": {"actuator": "bam_xl330", "trunk_body": "trunk_base", "freejoint": "trunk_base_freejoint",
                "gyro_sensor": "imu_ang_vel", "accel_sensor": "imu_accel", "home_z": 0.125},
        "servos": "14× Dynamixel XL330 (bus)",
        "link": "bus",
        "policies": {"walk.nnm": "MLP 61-512-256-128-14 (2048 env x 2000 iterazioni), int8 per riga "
                                 "ricavato dall'ONNX float32 validato. Nell'ambiente a contratto: in piedi 10 s, "
                                 "avanti 10 s (~0,10 m/s a comando 0,3), rotazione 10 s, nessuna caduta."},
        "notes": [
            "Integrazione di riferimento: SITL, batteria HIL 8/8 e HIL hardware su Pixhawk 6C Mini con la build float32.",
            "Il firmware comanda 14 funzioni servo (Scripting1..14). Il robot reale richiede un backend bus "
            "Dynamixel nel firmware, non ancora scritto; SITL e HIL usano il trasporto SIM_JSON / MAVLink.",
        ],
    },
    "microban": {
        "index": 1,
        "class": "biped",
        "display_name": "Microban",
        "maker": "Rhoban",
        "status": "policy-upstream",
        "joint_names": [
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch",
            "right_ankle_roll",
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch",
            "left_ankle_roll",
        ],
        "q0": [0.0, -10 * DEG, -20 * DEG, 0.0, -5 * DEG, -10 * DEG, 0.0, 0.0, 5 * DEG,
               0.0, 10 * DEG, -20 * DEG, 0.0, 5 * DEG, -10 * DEG, 0.0, 0.0, -5 * DEG],
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/Rhoban/mjlab_microban",
            "license": "Apache-2.0 (software); repo hardware GPL-3.0 / CC BY-NC-SA 4.0",
            "sim_model": "src/mjlab_microban/robot/microban/scene.xml",
            "hardware_repo": "https://github.com/Rhoban/microban",
            "cad": "https://github.com/Rhoban/microban/tree/main/cad (stl/, step/) e Onshape "
                   "https://cad.onshape.com/documents/d424992a192a8ce34ffce163",
            "bom": "https://github.com/Rhoban/microban/tree/main/docs (bom.md, printing.md, assembly.md)",
            "training": "mjlab 1.3.0 + rsl_rl PPO, modello attuatore BAM XL330 (kp_fw 125)",
            "published_policy": "https://github.com/Rhoban/microban/blob/main/src/agents/walk.onnx",
            "policy_url": "https://raw.githubusercontent.com/Rhoban/microban/main/src/agents/walk.onnx",
            "policy_name": "walk",
        },
        # the MJCF imu site is rotated w.r.t. the trunk; the env adds an IMU aligned with the trunk,
        # which is what the autopilot reports once AHRS_ORIENTATION matches its mounting
        "sim": {"actuator": "bam_xl330", "bam_kp_fw": 125.0, "trunk_body": "trunk",
                "freejoint": "trunk_freejoint", "home_z": 0.168},
        "servos": "19× Dynamixel XL330-M288-T (bus); la testa non è comandata dalla policy",
        "link": "bus",
        "policies": {"walk.nnm": "walk.onnx pubblicato da Rhoban (MLP 63-512-256-128-18, stesso contratto "
                                 "NNMixer) convertito in int8 per riga. Nell'ambiente a contratto: in piedi 10 s, "
                                 "avanti cade a 6,8 s, rotazione cade a 1 s. Con la gravità esatta del "
                                 "simulatore regge 10 s in avanti: la policy è stata addestrata senza il filtro "
                                 "IMU dell'autopilota. Da rifinire con --init-onnx prima dell'uso."},
        "notes": [
            "L'osservazione dell'ONNX è gyro, gravità proiettata, q−q0, q̇, azione precedente, twist: il formato "
            "NNMixer con 18 giunti, quindi la policy pubblicata si converte senza riaddestrarla.",
            "18 giunti superano le 16 funzioni servo Scripting consecutive: il firmware rifiuta questa "
            "topologia finché non esiste un backend bus Dynamixel. Simulazione e training funzionano già.",
        ],
    },
    "zeroth": {
        "index": 2,
        "class": "biped",
        "display_name": "Zeroth-01",
        "maker": "Zeroth Robotics / K-Scale Labs",
        "status": "needs-model",
        "joint_names": [
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee_pitch", "right_ankle_pitch",
            "right_ankle_roll",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee_pitch", "left_ankle_pitch",
            "left_ankle_roll",
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_roll", "left_gripper_roll",
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_roll", "right_gripper_roll",
        ],
        "q0": [0.0, -0.1, -0.4, -0.8, -0.4, -0.1,
               0.0, 0.1, -0.4, -0.8, -0.4, 0.1,
               0.0, 0.2, -0.2, 0.0,
               0.0, -0.2, 0.2, 0.0],
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/zeroth-robotics/zeroth-bot",
            "license": "MIT",
            "sim_model_note": "nessun MJCF nel repo: il task ksim lo scarica con "
                              "ksim.get_mujoco_model_path('zbot', name='robot') (pip install ksim); "
                              "salvarlo come robots/zeroth/robot/scene.xml",
            "cad": "Onshape https://cad.onshape.com/documents/cacc96f8a7850b951e7aa69a",
            "bom": "https://docs.kscale.dev/robots/zeroth-01/bom/",
            "training": "ksim (MuJoCo / JAX), policy ricorrente esportata in .kinfer",
            "published_policy": "release V0.2.1 ppo_standing.pt / ppo_walking.pt (pipeline più vecchia, "
                                "osservazione diversa); non convertibile",
        },
        "sim": {"actuator": "position", "home_z": 0.40},
        "servos": "servo Feetech STS3250 su bus seriale (la documentazione ne indica 16; il task ksim comanda 20 giunti)",
        "link": "bus",
        "policies": {},
        "notes": [
            "q0 è JOINT_BIASES di ksim-gym-zbot/train.py.",
            "L'osservazione ksim (quaternione, yaw assoluto, altezza della base) non è il contratto NNMixer: "
            "la policy va addestrata con tools/robots/train_velocity.py o mjlab.",
            "20 giunti superano le 16 funzioni servo Scripting: sull'autopilota serve il backend bus.",
        ],
    },
    "bimo": {
        "index": 3,
        "class": "biped",
        "display_name": "Bimo",
        "maker": "Mekion",
        "status": "needs-model",
        "joint_names": ["RHip", "LHip", "RShoulder", "LShoulder", "RKnee", "LKnee", "RAnkle", "LAnkle"],
        "q0": [-30 * DEG, -30 * DEG, 0.0, 0.0, 60 * DEG, 60 * DEG, 30 * DEG, 30 * DEG],
        "extra_cmd_dim": 0,
        "rate_hz": 25,
        "command_ranges": {"vx": [-0.2, 0.2], "vy": [0.0, 0.0], "wz": [-0.8, 0.8]},
        "upstream": {
            "repo": "https://github.com/mekion/the-bimo-project",
            "license": "Apache-2.0",
            "sim_model_note": "solo USD Isaac Lab (IsaacLab/bimo/assets/Bimo.usd); convertire USD -> MJCF "
                              "e salvarlo come robots/bimo/robot/scene.xml",
            "cad": "non ancora pubblicato (upstream: coming soon)",
            "bom": "non ancora pubblicata; per il setup MCU/README.md e BimoAPI/README.md",
            "training": "Isaac Lab + rsl_rl PPO, distillata in uno studente [64, 32] su RP2040",
            "published_policy": "nessuna",
        },
        "sim": {"actuator": "position", "home_z": 0.38},
        "servos": "8× Feetech STS3215 12 V (bus)",
        "link": "bus",
        "policies": {},
        "notes": [
            "L'upstream gira a 20 Hz, che non divide i 50 Hz del task AP_NNMixer: si addestra a 25 Hz.",
            "Le azioni upstream sono incrementi del comando giunto; il contratto NNMixer usa offset da q0, "
            "quindi la policy va riaddestrata.",
        ],
    },
    "legolas": {
        "index": 4,
        "class": "biped",
        "display_name": "Legolas",
        "maker": "daviddoo02",
        "status": "needs-model",
        "joint_names": ["R_Hip_1", "R_Hip_2", "R_Thigh", "R_Foreleg", "R_Servo",
                        "L_Hip_1", "L_Hip_2", "L_Thigh", "L_Foreleg", "L_Servo"],
        "q0": [0.0] * 10,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.1, 0.1], "wz": [-0.8, 0.8]},
        "upstream": {
            "repo": "https://github.com/daviddoo02/Legolas-an-open-source-biped",
            "license": "MIT",
            "sim_model": "Mujoco xml/Working Mk 5 - Old model - Demo only/CMU Mk 5.xml",
            "sim_model_note": "MJCF dimostrativo: giunto libero commentato, gravità 0, ctrlrange dei motori ±1e-5, "
                              "gambe a catena chiusa (4 vincoli connect); va sistemato prima del training e "
                              "salvato come robots/legolas/robot/scene.xml",
            "cad": "https://github.com/daviddoo02/Legolas-an-open-source-biped/tree/main/CAD "
                   "(SolidWorks V1-V3, STL in CAD/Legolas/V2/STLs)",
            "bom": "lista materiali nel README upstream; guida di montaggio non ancora pubblicata",
            "training": "nessuno upstream (controllore IK del passo su ROS)",
            "published_policy": "nessuna",
        },
        "sim": {"actuator": "position", "home_z": 0.6},
        "servos": "10 servo hobby PWM (8× 40 kg, 2× 80 kg), upstream tramite PCA9685",
        "link": "pwm",
        "policies": {},
        "notes": [
            "q0 non è pubblicata: va definita la posa di stand sul MJCF sistemato e scritta in catalog.py.",
            "I servo PWM si collegano direttamente alle uscite dell'autopilota (10 ≤ 16 funzioni servo); manca "
            "ancora in robot.bin una calibrazione per giunto (verso, centro, rad/µs).",
        ],
    },
    "upkie": {
        "index": 5,
        "class": "biped",
        "display_name": "Upkie (wheeled biped)",
        "maker": "upkie / Stéphane Caron",
        "status": "needs-firmware",
        "joint_names": ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"],
        "q0": [0.0] * 6,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.8, 0.8], "vy": [0.0, 0.0], "wz": [-1.5, 1.5]},
        "upstream": {
            "repo": "https://github.com/upkie/upkie",
            "license": "Apache-2.0",
            "model_repo": "https://github.com/MarcDcls/mjlab_upkie",
            "sim_model": "src/mjlab_upkie/robot/upkie/scene.xml",
            "cad": "https://github.com/upkie/upkie_parts (FreeCAD, STL, 3MF)",
            "bom": "https://upkie.github.io/upkie/build-your-own.html",
            "training": "mjlab 1.3.0 + rsl_rl (MarcDcls/mjlab_upkie)",
            "published_policy": "MarcDcls/mjlab_upkie logs/rsl_rl/upkie_velocity/bests/default.onnx "
                                "(osservazione con quaternione del tronco, ruote in velocità); non convertibile",
        },
        "sim": {"actuator": "position", "home_z": 0.343},
        "servos": "4× mjbots qdd100 (anche, ginocchia), 2× moteus + mj5208 (ruote), CAN-FD",
        "link": "can",
        "policies": {},
        "notes": [
            "Le ruote ricevono comandi di velocità. AP_NNMixer tratta ogni azione come offset di posizione, "
            "quindi Upkie richiede nel firmware un tipo di azione per giunto.",
        ],
    },
    "rex": {
        "index": 6,
        "class": "quadruped",
        "display_name": "Rex / SpotMicro",
        "maker": "nicrusso7 (SpotMicroAI community design)",
        "status": "needs-training",
        "joint_names": [
            "motor_front_left_shoulder", "motor_front_left_leg", "foot_motor_front_left",
            "motor_front_right_shoulder", "motor_front_right_leg", "foot_motor_front_right",
            "motor_rear_left_shoulder", "motor_rear_left_leg", "foot_motor_rear_left",
            "motor_rear_right_shoulder", "motor_rear_right_leg", "foot_motor_rear_right",
        ],
        "q0": [0.0, -0.88643435, 1.30197369] * 4,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.3, 0.3], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/nicrusso7/rex-gym",
            "branch": "master",
            "license": "Apache-2.0",
            "urdf": "rex_gym/util/pybullet_data/assets/urdf/rex.urdf",
            "cad": "https://www.thingiverse.com/thing:3445283 e https://github.com/FlorianWilk/SpotMicroAI",
            "bom": "https://github.com/nicrusso7/rexctl/wiki/Mark-I",
            "training": "PyBullet + PPO TensorFlow 1 (ibrido open-loop + residuo)",
            "published_policy": "checkpoint TensorFlow rex_gym/policies/*/model.ckpt-* (osservazione di 4 "
                                "valori, azioni residue); non convertibili",
        },
        "sim": {"actuator": "position", "trunk_body": "base_link", "freejoint": "root",
                "gyro_sensor": "nnm_gyro", "accel_sensor": "nnm_accel", "home_z": 0.22,
                "position_kp": 8.0, "position_kv": 0.3, "force_limit": 1.1},
        "servos": "12× MG996R PWM",
        "link": "pwm",
        "policies": {},
        "notes": [
            "Solo URDF: fetch_upstream.py genera robots/rex/robot/scene.xml (giunto libero, pavimento, IMU, "
            "attuatori di posizione). Guadagni e limiti di coppia degli attuatori sono stime da verificare.",
            "I servo PWM si collegano direttamente alle uscite dell'autopilota (12 ≤ 16); manca ancora in "
            "robot.bin la calibrazione per giunto.",
        ],
    },
    "yertle": {
        "index": 7,
        "class": "quadruped",
        "display_name": "Yertle",
        "maker": "Jerome Graves",
        "status": "needs-training",
        "joint_names": [
            "lf_shoulder", "lf_thigh", "lf_shin",
            "rf_shoulder", "rf_thigh", "rf_shin",
            "lb_shoulder", "lb_thigh", "lb_shin",
            "rb_shoulder", "rb_thigh", "rb_shin",
        ],
        "q0": [0.0, -0.6, 0.9] * 4,
        "extra_cmd_dim": 0,
        "rate_hz": 50,
        "command_ranges": {"vx": [-0.4, 0.4], "vy": [-0.2, 0.2], "wz": [-1.0, 1.0]},
        "upstream": {
            "repo": "https://github.com/Jerome-Graves/yertle",
            "license": "MIT",
            "urdf": "simulation/yertle.urdf",
            "cad": "https://github.com/Jerome-Graves/yertle/tree/main/design (CAD/Yertle_Single_v2.step, STL/)",
            "bom": "https://github.com/Jerome-Graves/yertle/blob/main/design/README.md",
            "training": "SB3 PPO su PyBullet; Isaac Lab + rsl_rl",
            "published_policy": "nessuna",
        },
        "sim": {"actuator": "position", "trunk_body": "base_link", "freejoint": "root",
                "gyro_sensor": "nnm_gyro", "accel_sensor": "nnm_accel", "home_z": 0.25,
                "position_kp": 15.0, "position_kv": 0.5, "force_limit": 3.4},
        "servos": "12× SPT5435LV-180W 35 kg PWM, upstream tramite PCA9685 su ESP32",
        "link": "pwm",
        "policies": {},
        "notes": [
            "L'osservazione upstream (48) è il formato NNMixer più la velocità lineare della base, che "
            "l'autopilota non misura; il contratto NNMixer la toglie (45).",
            "I servo PWM si collegano direttamente alle uscite dell'autopilota (12 ≤ 16); manca ancora in "
            "robot.bin la calibrazione per giunto.",
        ],
    },
}

STATUS_TEXT = {
    "policy": "policy int8 disponibile; la stessa rete in float32 è validata in SITL e HIL",
    "policy-upstream": "policy upstream convertita in int8; simulazione e training pronti",
    "needs-training": "scena MuJoCo generata dall'URDF; policy da addestrare",
    "needs-model": "manca una scena MuJoCo pronta per il training",
    "needs-firmware": "serve un tipo di azione per giunto nel firmware (ruote in velocità)",
}

LINK_TEXT = {
    "bus": "servo su bus seriale: serve il backend bus nel firmware (non ancora scritto)",
    "pwm": "servo PWM: collegabili alle uscite dell'autopilota",
    "can": "attuatori CAN-FD mjbots: serve un backend dedicato",
}
