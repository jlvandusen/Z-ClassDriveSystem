# Z-Class BB-8 Drive System — hardware

The mechanics, PCBs, and build documentation for the **Z-Class** BB-8 drive: a
chain-driven ball-bot drive descended from Joe's Drive V2, with geared
side-to-side steering, a low-profile flywheel, and a single all-in-one mainboard.
This repo is the **hardware** half of the project — print it, order it, wire it,
assemble it.

> ## ⚡ The software lives in [Z-ClassControlSystem](https://github.com/jlvandusen/Z-ClassControlSystem)
>
> **All firmware, the `bb8` build/flash/tune CLI, and every operating document for
> this drive are maintained in
> [Z-ClassControlSystem](https://github.com/jlvandusen/Z-ClassControlSystem) — that
> repo is primary for all control and software supporting this drive.**
>
> - **Install the tooling** (Windows, no admin): grab a
>   [release](https://github.com/jlvandusen/Z-ClassControlSystem/releases) —
>   **`Setup-BASIC`** to drive the droid (prebuilt firmware, bundled flashers, no
>   toolchain or git) or **`Setup-MAX`** to modify the firmware (source + compile
>   toolchain).
> - **Read the docs** on the [wiki](https://github.com/jlvandusen/Z-ClassControlSystem/wiki):
>   [First-Time Setup](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/First-Time-Setup) ·
>   [How-To Guide](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/How-To-Guide) ·
>   [Runbook](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/Runbook) ·
>   [Rig Tuning](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/Rig-Tuning) ·
>   [Assembly (Drive)](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/Assembly-Drive)
>
> The legacy v9.15 firmware that used to live in `SourceFiles/` has been removed —
> it was fully superseded by the RC4 firmware in the control repo
> (`SourceFiles/README.md` maps each old sketch to its replacement).

![Z-Class System v2](Z-Class%20System%20v2.png)

## The drive at a glance

| Feature | Description |
|---|---|
| **Chain main drive** | The shell is driven by chain (planetary gearmotor → sprockets) — no slipping, unlike friction/belt drives. A geared main-drive variant is included (`STLs/DriveSystem_Gear`). |
| **Geared S2S** | Side-to-side steering tilts the inner frame through a gear set (with a geared position pot for closed-loop control), similar in spirit to Bruton's design. |
| **v2 flywheel** | Very low profile — sits above the battery compartment, allowing out-of-ball testing and better weight distribution. |
| **All-in-one PCB** | The mainboard has buck converters built in with all connections on pin headers and JST-XH — battery in, 5 V / 6 V / 9 V rails out. |
| **Encoder dome spin** | The dome rotation motor's built-in encoder gives exact directional control. |
| **Dome by radio** | Body ↔ dome over ESP-NOW — no WiFi network, no pairing. The dome's USB port doubles as a wireless console into the sealed ball. |
| **Audio** | DFPlayer Mini on the body node — an SD card of `MP3/NNNN.mp3` clips, every cue remappable at runtime (see the control repo's How-To §5). |

## Electronics

Four boards, all Adafruit-form-factor; what each does, how they wire together,
and everything about flashing/tuning them is in the control repo:

| Node | Board | Firmware (Z-ClassControlSystem) |
|---|---|---|
| drive (master) | ESP32 HUZZAH32 Feather | `firmware/ESP32_DRIVE_RC4` — gamepads over Bluetooth, 100 Hz balance PIDs |
| body | Feather 32u4 | `firmware/32U4_DRIVE_RC4` — dome tilt servos, dome spin, DFPlayer audio |
| imu | Trinket M0 + MPU6050 | `firmware/TrinketM0_MPU_RC4` — Kalman pitch/roll at 100 Hz |
| dome | ESP32 HUZZAH32 Feather | `firmware/ESP32_DOME_RC4` — lights, battery, wireless console bridge |

## What's in this repo

| Folder / file | Contents |
|---|---|
| `STLs/` | All printable drive parts — frame slices, gantry, S2S arm + gears, flywheel arm + gears, head-tilt masts (plain / Hall-sensor variants), battery harness, ballast, couplers. `STLs/DriveSystem_Gear/` is the geared main-drive alternative (47/94/26-tooth gears, casings). |
| `Frame/` | Drive hub / axle mounts and the CNC circle patterns for the frame (`bb8-cnc-circles-painted.zip`). |
| `BMM/` | Shaft and bearing adapters — Halo BMM prints for 8 mm and 9.525 mm shafts (Joe's Drive Mk2 compatible), D-shaft, 10→12 mm adapter. |
| `DMM/` | Ready-to-print G-code for the DMM bearing and 50 mm D-shaft. |
| `Support Hardware Step Files/` | CAD for the bought parts: NeveRest 60 and RobotZone planetary gearmotors, 12 V worm-gear motor (S2S), ServoCity 1600-series, 608ZZ bearing. |
| `PCB/` | Fabrication files for every board revision — see below. |
| `Z-Drive BOM.xlsx` | Bill of materials. |
| `JoeDriveV2_v9.15.pdf` | Wiring documentation for the v9.15 mainboard (as built). |
| `v2 Drive Assembly.docx` | Mechanical assembly notes. A newer assembly walkthrough generated from the Fusion 360 model lives on the control repo's wiki: [Assembly (Drive)](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/Assembly-Drive). |
| `ESP32 Proposed Solution v76.pdf` | Original electronics design proposal (historical). |
| `BB8_Operator_Card_PID_Tuning.pdf` | **Legacy** — written for the v9.15 firmware; RC4 changed the PID units. Current tuning procedure: [Rig Tuning](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/Rig-Tuning). |

## PCBs (`PCB/`)

| Zip | Board | Status |
|---|---|---|
| `BB8 Mainboard v9.15_2025-12-30.zip` | All-in-one mainboard v9.15 | **As built** — the boards driving the droid today. Wiring: `JoeDriveV2_v9.15.pdf`. Design review: [v9 PCB Analysis](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/v9-PCB-Analysis). |
| `ZDrive_v10_compact_GERBERS_r7_BOM_CPL.zip` | v10 compact mainboard (r7) — gerbers + BOM + CPL as ordered from JLCPCB | **BETA, at fab** — next-generation single board; works with the gear drive. Design doc: [v10 Board Design](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/v10-Board-Design). |
| `bb8 DOME v8.2_2024-05-05.zip` | Dome board v8.2 | As built. |
| `bb8 IMU v8.2_2024-05-05.zip` | IMU carrier v8.2 | As built. |

## Heritage

The Z-Class drive shares its lineage with **Joe's Drive V2** — several printed
parts remain Mk2-compatible (see `BMM/`). Thanks to Joe and the BB-8 builders
community.
