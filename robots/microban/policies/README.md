# Policy per Microban

Solo policy per `microban` (18 giunti, osservazione 63). Sulla microSD: `/APM/nnm/microban/policies/`.

- `walk.nnm` — walk.onnx pubblicato da Rhoban (MLP 63-512-256-128-18, stesso contratto NNMixer) convertito in int8 per riga. Nell'ambiente a contratto: in piedi 10 s, avanti cade a 6,8 s, rotazione cade a 1 s. Con la gravità esatta del simulatore regge 10 s in avanti: la policy è stata addestrata senza il filtro IMU dell'autopilota. Da rifinire con --init-onnx prima dell'uso.
