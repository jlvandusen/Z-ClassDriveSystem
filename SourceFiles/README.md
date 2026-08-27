# Moved — the firmware lives in Z-ClassControlSystem

The sketches that used to live here (`ESP32_Primary_v9_15`, `32u4_Secondary_v9_15`,
`ESP32_Dome_v9.15`, `TrinketM0_IMU_v9.15`) were the **legacy v9.15 firmware**.
They have been removed — fully superseded and no longer maintained (they remain
in this repo's git history if you ever need them).

**All current firmware, tooling, and documentation is in
[Z-ClassControlSystem](https://github.com/jlvandusen/Z-ClassControlSystem):**

- `firmware/ESP32_DRIVE_RC4` — drive master (replaces ESP32_Primary)
- `firmware/32U4_DRIVE_RC4` — body node (replaces 32u4_Secondary)
- `firmware/ESP32_DOME_RC4` — dome (replaces ESP32_Dome)
- `firmware/TrinketM0_MPU_RC4` — IMU (replaces TrinketM0_IMU)
- the `bb8` CLI — build, flash, monitor, tune, self-update

Start here: [Releases](https://github.com/jlvandusen/Z-ClassControlSystem/releases)
(`Setup-BASIC` to drive, `Setup-MAX` to develop) and the
[First-Time Setup guide](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/First-Time-Setup).

Why RC4 replaced v9.15: a 112-finding review of the old control code —
[RC4 Review and Fixes](https://github.com/jlvandusen/Z-ClassControlSystem/wiki/RC4-Review-and-Fixes).
