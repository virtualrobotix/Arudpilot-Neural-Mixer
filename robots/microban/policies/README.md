# Policy per Microban

Solo policy per `microban` (18 giunti, osservazione 63). Sulla microSD: `/APM/nnm/microban/policies/`.

- `walk.nnm` — walk.onnx pubblicato da Rhoban (MLP 63-512-256-128-18, stesso contratto NNMixer) convertito in int8 per riga. Nell'ambiente a contratto: in piedi 10 s, avanti cade a 6,8 s, rotazione cade a 1 s. Con la gravità esatta del simulatore regge 10 s in avanti: la policy è stata addestrata senza il filtro IMU dell'autopilota. Da rifinire con --init-onnx prima dell'uso.
- `walk_md.nnm` — addestrata da zero sul contratto ArduPilot in int8 (QAT): reward e curriculum del task MicroDuck più premi sul passo (appoggio singolo, piede sollevato, alternanza, simmetria), 512 env × 3000 iterazioni, checkpoint 2700 (punteggio 0,64). Sopravvivenza 100% in tutte le modalità; avanti/indietro ~0,16-0,19 m/s a comando 0,3; rotazione 0,68-0,87 rad/s a comando 0,8; laterale 0,04 m/s a comando 0,2; passo alternato e simmetrico (3-4 cm, 0,27-0,29 s per piede). W&B mjlab_microban/h5e7djou.
