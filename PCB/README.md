# PCB fabrication files

| Zip | Board | Status |
|---|---|---|
| `BB8 Mainboard v9.15_2025-12-30.zip` | All-in-one mainboard v9.15 | **As built** — the boards driving the droid today. Wiring: [`../JoeDriveV2_v9.15.pdf`](../JoeDriveV2_v9.15.pdf). Design review: [v9 PCB Analysis](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/v9-PCB-Analysis). |
| `ZDrive_v10_compact_GERBERS_r7_BOM_CPL.zip` | v10 compact mainboard (r7) — gerbers + BOM + CPL as ordered from JLCPCB | **BETA, at fab** — next-generation single board; works with the gear drive. Design doc: [v10 Board Design](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/v10-Board-Design). |
| `bb8 DOME v8.2_2024-05-05.zip` | Dome board v8.2 | As built. |
| `bb8 IMU v8.2_2024-05-05.zip` | IMU carrier v8.2 | As built. |

The firmware these boards run — and the `bb8` tool that flashes and tunes them —
lives in [Z-ClassControlSystem](https://github.com/jlvandusen/Z-ClassControlSystem).
KiCad sources for the v10 board are on that repo's `v10` branch under `hardware/`.
