# Catalogo robot NNMixer

AP_NNMixer separa **topologia del robot** (fissata al boot) e **policy** (scambiabili solo dentro la cartella di quel robot).

- Repo: `robots/<id>/robot/` + `robots/<id>/policies/`
- MicroSD: `/APM/nnm/<id>/robot.bin` + `/APM/nnm/<id>/policies/<nome>.nnm`

Parametri: `NNM_ROBOT` sceglie la directory al boot; `NNM_POLICY` sceglie un file solo in `policies/` di quel robot. Una policy di un altro `id` viene rifiutata (controllo su `robot_id` e dimensioni).

## Bipedi / umanoidi

| Robot | Stato | Upstream |
|---|---|---|
| [MicroDuck (Pollen / Hugging Face)](microduck.md) | `integrated` | [microduck](https://github.com/pollen-robotics/microduck_rl) |
| [Microban](microban.md) | `pipeline-ready` | [microban](https://github.com/Rhoban/mjlab_microban) |
| [Zeroth-01](zeroth.md) | `needs-mjcf` | [zeroth](https://github.com/zeroth-robotics/zeroth-bot) |
| [Bimo](bimo.md) | `needs-mjcf` | [bimo](https://github.com/mekion/the-bimo-project) |
| [Legolas](legolas.md) | `needs-mjcf` | [legolas](https://github.com/daviddoo02/Legolas-an-open-source-biped) |
| [Upkie (wheeled biped)](upkie.md) | `needs-mjcf` | [upkie](https://github.com/upkie/upkie) |

## Quadrupedi

| Robot | Stato | Upstream |
|---|---|---|
| [Rex / SpotMicro](rex.md) | `needs-mjcf` | [rex](https://github.com/nicrusso7/rex-gym) |
| [Yertle](yertle.md) | `needs-mjcf` | [yertle](https://github.com/Jerome-Graves/yertle) |

## Stati

- `integrated` — scena, profilo e policy MLP già usati dal firmware
- `pipeline-ready` — upstream già allineato a MjLab/ONNX; manca il training locale
- `needs-mjcf` — va portata la scena in MuJoCo e verificata a policy zero

CAD e STL restano upstream (Apache-2.0 / MIT): le pagine linkano, non vendono l’intero albero.
