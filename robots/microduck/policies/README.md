# Policy per MicroDuck

Solo policy per `microduck` (14 giunti, osservazione 61). Sulla microSD: `/APM/nnm/microduck/policies/`.

- `walk.nnm` — MLP 61-512-256-128-14 (2048 env x 2000 iterazioni), int8 per riga ricavato dall'ONNX float32 validato. Nell'ambiente a contratto: in piedi 10 s, avanti 10 s (~0,10 m/s a comando 0,3), rotazione 10 s, nessuna caduta.
