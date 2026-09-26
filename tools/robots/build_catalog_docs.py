# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Robot pages (docs/robots/*.md) and policy indexes, generated from tools/robots/catalog.py."""

from __future__ import annotations

from pathlib import Path
from urllib.parse import quote

CLASS_IT = {"biped": "bipede", "quadruped": "quadrupede"}


def _obs_dim(r: dict) -> int:
    return 3 + 3 + 3 * len(r["joint_names"]) + 3 + r.get("extra_cmd_dim", 0)


def _blob(repo: str, path: str, branch: str = "main") -> str:
    return f"{repo}/blob/{branch}/{quote(path)}"


def _link(text: str) -> str:
    """Turn bare URLs inside a catalog string into markdown links."""
    out = []
    for w in text.split(" "):
        if w.startswith("http"):
            tail = ""
            while w and w[-1] in ",;)":
                tail = w[-1] + tail
                w = w[:-1]
            out.append(f"[{w.split('://', 1)[1]}]({w}){tail}")
        else:
            out.append(w)
    return " ".join(out)


def _policies_on_disk(repo_root: Path, rid: str) -> list[Path]:
    return sorted((repo_root / "robots" / rid / "policies").glob("*.nnm"))


def robot_page(rid: str, r: dict, status_text: dict, link_text: dict, repo_root: Path) -> str:
    u = r["upstream"]
    n = len(r["joint_names"])
    od = _obs_dim(r)
    branch = u.get("branch", "main")
    model_repo = u.get("model_repo", u["repo"])
    lines = [
        f"# {r['display_name']}",
        "",
        f"[Catalogo robot](README.md) · [Training compatibile con ArduPilot](training.md) · "
        f"[README](../../README.it.md)",
        "",
        "| | |",
        "|---|---|",
        f"| Id (cartella) | `{rid}` |",
        f"| `NNM_ROBOT` | **{r['index']}** |",
        f"| Classe | {CLASS_IT[r['class']]} |",
        f"| Progetto | {r['maker']} |",
        f"| Stato | {status_text[r['status']]} |",
        f"| Giunti comandati | {n} |",
        f"| Osservazione | {od} valori |",
        f"| Frequenza policy | {r['rate_hz']} Hz |",
        f"| Attuatori | {r['servos']} |",
        f"| Collegamento | {link_text[r['link']]} |",
        f"| Licenza upstream | {u.get('license', 'vedi upstream')} |",
        "",
        "## Repo originale e file di base",
        "",
        f"- Repository: [{u['repo'].split('://', 1)[1]}]({u['repo']})",
    ]
    if u.get("model_repo"):
        lines.append(f"- Modello MuJoCo e training: [{model_repo.split('://', 1)[1]}]({model_repo})")
    if u.get("hardware_repo"):
        lines.append(f"- Hardware: [{u['hardware_repo'].split('://', 1)[1]}]({u['hardware_repo']})")
    if u.get("sim_model"):
        lines.append(f"- Scena MuJoCo upstream: [`{u['sim_model']}`]({_blob(model_repo, u['sim_model'], branch if model_repo == u['repo'] else 'main')})")
    if u.get("urdf"):
        lines.append(f"- URDF upstream: [`{u['urdf']}`]({_blob(u['repo'], u['urdf'], branch)})")
    lines += [
        f"- CAD e parti stampabili: {_link(u.get('cad', 'non trovato'))}",
        f"- BOM e montaggio: {_link(u.get('bom', 'non trovato'))}",
        f"- Training upstream: {u.get('training', 'non trovato')}",
        f"- Policy upstream: {_link(u.get('published_policy', 'nessuna'))}",
        "",
        "Per scaricare il repo originale e preparare la scena usata da simulazione e training:",
        "",
        "```bash",
        f".venv/bin/python tools/robots/fetch_upstream.py --robot {rid}",
        "```",
        "",
    ]
    if u.get("sim_model_note"):
        lines += [f"Nota sul modello: {u['sim_model_note']}.", ""]
    if u.get("urdf"):
        lines += [
            f"Lo script compila l'URDF con MuJoCo e scrive `robots/{rid}/robot/scene.xml` con giunto libero, "
            "pavimento, IMU sul tronco e attuatori di posizione.",
            "",
        ]
    lines += [
        "## Topologia",
        "",
        f"File: [`robots/{rid}/robot/profile.json`](../../robots/{rid}/robot/profile.json) → "
        f"`robot.bin` sulla microSD. Ordine dei giunti = ordine di osservazione, azione e uscite servo.",
        "",
        "| # | Giunto | q0 (rad) | Uscita servo |",
        "|---:|---|---:|---|",
    ]
    for i, (name, q) in enumerate(zip(r["joint_names"], r["q0"])):
        fn = 94 + i
        out = f"`SERVO{i + 1}_FUNCTION {fn}`" if fn <= 109 else "oltre Scripting16: serve il backend bus"
        lines.append(f"| {i} | `{name}` | {q:+.4f} | {out} |")
    extra = r.get("extra_cmd_dim", 0)
    lines += [
        "",
        f"Osservazione ({od}): gyro FLU 3, gravità FLU 3, q−q0 {n}, q̇ {n}, azione precedente {n}, "
        f"twist vx vy ωz 3" + (f", comandi testa/corpo {extra} (zero sull'autopilota)" if extra else "") + ".",
        f"Azione ({n}): offset in radianti, `q_target = q0 + azione`.",
        "",
        "## Architettura PPO",
        "",
        f"File: [`robots/{rid}/robot/ppo.yaml`](../../robots/{rid}/robot/ppo.yaml). Stessa rete di MicroDuck, "
        f"già provata su Pixhawk 6C: **{od} → 512 → 256 → 128 → {n}**, ELU, normalizzazione dell'osservazione "
        "incorporata. Pesi int8 per riga addestrati sulla griglia int8 dalla prima iterazione (QAT), "
        "attivazioni float32. PPO: 2048 ambienti × 24 passi, 5 epoche, 4 minibatch, lr 1e-3 adattivo "
        "(KL 0,01), γ 0,99, λ 0,95, clip 0,2.",
        "",
        "## Policy",
        "",
        f"Cartella: [`robots/{rid}/policies/`](../../robots/{rid}/policies/) → sulla microSD "
        f"`/APM/nnm/{rid}/policies/`. `NNM_POLICY` = indice del file in ordine alfabetico.",
        "",
    ]
    on_disk = _policies_on_disk(repo_root, rid)
    if on_disk:
        lines += ["| File | Dimensione | Descrizione |", "|---|---:|---|"]
        for p in on_disk:
            desc = r["policies"].get(p.name, "")
            lines.append(f"| `{p.name}` | {p.stat().st_size / 1024:.0f} KB | {desc} |")
        lines.append("")
    else:
        lines += ["Nessuna policy ancora: va addestrata (sezione successiva).", ""]
    lines += [
        "## Training compatibile con ArduPilot",
        "",
        "L'ambiente è già il deployment: fisica a 200 Hz come il loop dell'autopilota, gravità dal filtro IMU "
        "del firmware, azione tagliata a `NNM_ACT_MAX` e codificata in PWM, osservazione nell'ordine del "
        "profilo. Dettagli in [training.md](training.md).",
        "",
        "```bash",
        f".venv/bin/python tools/robots/nnm_env.py --robot {rid}                       # carica la scena, passo a policy zero",
        f".venv/bin/python tools/robots/train_velocity.py --robot {rid} --envs 8 --iters 200 --name walk",
    ]
    if r["policies"] and rid != "microduck" and u.get("policy_name"):
        lines.append(f".venv/bin/python tools/robots/train_velocity.py --robot {rid} "
                     f"--init-onnx robots/{rid}/policies/{u['policy_name']}.onnx --name walk_ap   "
                     "# rifinitura (ONNX scaricato da fetch_upstream.py)")
    lines += [
        f".venv/bin/python tools/robots/train_velocity.py --robot {rid} --eval robots/{rid}/policies/walk.nnm",
        "```",
        "",
        "Il trainer CPU serve per verifiche e rifiniture brevi. Per una policy completa (2048 ambienti × 2000 "
        "iterazioni) si usa mjlab su GPU con gli stessi due pezzi: `deploy_contract.py` per osservazione e "
        "azione, `nnm_qat.enable_qat()` sull'actor, `export_nnm_from_actor()` per il file.",
        "",
        "## Deploy sull'autopilota",
        "",
        "```bash",
        f"python tools/robots/pack_robot_bin.py --robot {rid}          # robot.bin dal profilo",
        f"# microSD: /APM/nnm/{rid}/robot.bin  e  /APM/nnm/{rid}/policies/*.nnm",
        "```",
    ]
    if n <= 16:
        lines += [
            "",
            f"Parametri: `NNM_ENABLE 1`, `NNM_ROBOT {r['index']}` (riavvio), `NNM_POLICY 0`, "
            f"`SERVO1..{n}_FUNCTION 94..{94 + n - 1}`, `INS_GYRO_FILTER 0`, `SCHED_LOOP_RATE 200`.",
        ]
    else:
        lines += [
            "",
            f"Con {n} giunti il firmware attuale rifiuta `robot.bin` (massimo 16 funzioni servo Scripting). "
            "Il deploy richiede il backend bus; simulazione, training e file `.nnm` sono già pronti.",
        ]
    lines += ["", "## Note", ""]
    lines += [f"- {x}" for x in r["notes"]]
    return "\n".join(lines) + "\n"


def index_page(robots: dict, status_text: dict, link_text: dict, repo_root: Path) -> str:
    lines = [
        "# Robot supportati da AP_NNMixer",
        "",
        "[README](../../README.it.md) · [Training compatibile con ArduPilot](training.md)",
        "",
        "Ogni robot ha una **topologia** fissata al boot (`NNM_ROBOT`) e **policy** proprie, scambiabili a "
        "caldo solo dentro la sua cartella (`NNM_POLICY`). Una policy di un altro robot viene rifiutata dal "
        "firmware (controllo su `robot_id` e dimensioni).",
        "",
        "Tutti i robot usano la stessa architettura PPO di MicroDuck (MLP 512-256-128 ELU, int8 per riga); "
        "cambiano solo ingresso e uscita.",
        "",
        "## Configurazioni disponibili",
        "",
        "| `NNM_ROBOT` | Robot | Classe | Giunti | Osservazione | Hz | Collegamento | Policy | Stato |",
        "|---:|---|---|---:|---:|---:|---|---|---|",
    ]
    for rid, r in sorted(robots.items(), key=lambda kv: kv[1]["index"]):
        pol = ", ".join(f"`{p.name}`" for p in _policies_on_disk(repo_root, rid)) or "—"
        lines.append(
            f"| {r['index']} | [{r['display_name']}]({rid}.md) | {CLASS_IT[r['class']]} | {len(r['joint_names'])} | "
            f"{_obs_dim(r)} | {r['rate_hz']} | {r['link']} | {pol} | {status_text[r['status']]} |")
    lines += [
        "",
        "Collegamento: " + "; ".join(f"`{k}` = {v}" for k, v in link_text.items()) + ".",
        "",
        "## Struttura dei file (repo e microSD)",
        "",
        "```",
        "robots/<id>/robot/profile.json   topologia              ->  /APM/nnm/<id>/robot.bin",
        "robots/<id>/robot/ppo.yaml       architettura PPO + ambiente di training",
        "robots/<id>/robot/scene.xml      scena MuJoCo (generata dall'URDF quando serve)",
        "robots/<id>/policies/*.nnm       policy int8            ->  /APM/nnm/<id>/policies/*.nnm",
        "```",
        "",
        "`tools/robots/catalog.py` è l'unica fonte dei dati; `tools/robots/build_catalog.py` rigenera profili, "
        "`ppo.yaml` e queste pagine.",
        "",
        "## Cosa manca per l'hardware",
        "",
        "- Robot con servo su bus (Dynamixel, Feetech): un backend bus nel firmware. Oggi le uscite sono "
        "funzioni servo Scripting1..16, sufficienti per SITL, HIL e servo PWM.",
        "- Robot con servo PWM: una calibrazione per giunto (verso, centro, rad/µs) in `robot.bin`.",
        "- Upkie: un tipo di azione per giunto (le ruote vanno in velocità).",
    ]
    return "\n".join(lines) + "\n"


def policies_readme(rid: str, r: dict, repo_root: Path) -> str:
    files = _policies_on_disk(repo_root, rid)
    lines = [f"# Policy per {r['display_name']}", "",
             f"Solo policy per `{rid}` ({len(r['joint_names'])} giunti, osservazione {_obs_dim(r)}). "
             f"Sulla microSD: `/APM/nnm/{rid}/policies/`.", ""]
    if files:
        lines += [f"- `{p.name}` — {r['policies'].get(p.name, '')}" for p in files]
    else:
        lines.append(f"Nessuna policy ancora. Addestrare con "
                     f"`tools/robots/train_velocity.py --robot {rid}` (vedi docs/robots/{rid}.md).")
    return "\n".join(lines) + "\n"


def write_docs(robots: dict, status_text: dict, link_text: dict, repo_root: Path) -> None:
    docs = repo_root / "docs" / "robots"
    docs.mkdir(parents=True, exist_ok=True)
    for rid, r in robots.items():
        (docs / f"{rid}.md").write_text(robot_page(rid, r, status_text, link_text, repo_root))
        (repo_root / "robots" / rid / "policies" / "README.md").write_text(policies_readme(rid, r, repo_root))
    (docs / "README.md").write_text(index_page(robots, status_text, link_text, repo_root))
    print(f"wrote docs/robots/README.md and {len(robots)} robot pages")
