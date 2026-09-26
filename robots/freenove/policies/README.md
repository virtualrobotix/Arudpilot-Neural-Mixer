# Policy per Freenove Robot Dog

Solo policy per `freenove` (12 giunti, osservazione 45). Sulla microSD: `/APM/nnm/freenove/policies/`.

- `walk_v7.nnm` — addestrata da zero sul contratto ArduPilot in int8 (QAT), ambiente walk_v7 (passo da cane, un comando per asse). Checkpoint dell'iterazione 3000 (15000 epoche PPO, punteggio 0,59). In valutazione sopravvive sempre: avanti 0,21 m/s a comando 0,15; indietro, laterale e rotazione non ancora seguiti.
