# Changelog

All notable changes to this project will be documented in this file. The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.1] — 2026-05-11

Initial driver scaffolding and the control pipeline (gamepad → motor PWM). Sensor, ML, watchdog, and setup-automation layers are planned for later releases.

### Added

- `ament_python` package skeleton (`package.xml`, `setup.py`, `setup.cfg`, resource marker)
- `gamepad_node` — reads configured axes from `/joy` and publishes a normalized command in `[-1, 1]` to `/gamepad_drive`
- `mux_node` — timer-driven (50 Hz) command arbitration on `/mux_out`:
  - LB held → forwards `/gamepad_drive`
  - RB held → forwards `/drive` (autonomy)
  - Neither / both → publishes zero
  - 500 ms `/joy` disconnect timeout → publishes zero
  - 500 ms upstream command staleness → publishes zero
- `throttle_node` — single source of truth for per-direction speed and steering caps; clamps and rescales `/mux_out` → `/motor`
- `pwm_node` — two-parameter servo calibration per axis (`center_pwm` + `magnitude_pwm`); maps `[-1, 1]` commands to Pololu Maestro pulses
- `maestro.py` — Pololu serial protocol library (verbatim port from v1)
- Per-node launch files (`gamepad.launch.py`, `mux.launch.py`, `throttle.launch.py`, `pwm.launch.py`) so the future watchdog can restart any one in isolation
- Top-level `teleop.launch.py` composing all four with `joy_node`
- Parameter YAMLs: `config/gamepad.yaml`, `mux.yaml`, `throttle.yaml`, `pwm.yaml`
- Project files: `README.md`, `LICENSE` (GPLv3), `.gitignore`, `.gitattributes`

### Design notes & migration from v1

- **Normalized `[-1, 1]` command convention** on every intermediate topic. Autonomy code publishing to `/drive` should target this range; v1 expected `[-0.25, 0.25]`.
- **Single tuning surface for top speed.** `max_speed_forward / max_speed_backward / max_steering` in `throttle.yaml` are the only place the effective top speed is set. v1 spread this across three nodes with three duplicated constants.
- **Two-step servo calibration in `pwm.yaml`.** Per axis: (1) find `center_pwm` at command = 0, (2) raise `magnitude_pwm` at command = +1 until visible saturation. Replaces v1's six interdependent parameters per axis.
- **Mux is timer-driven** at 50 Hz, not event-driven on `/joy` callbacks. Keeps the Maestro continuously fed and gives the future watchdog an unambiguous "mux alive" signal.
- **Mux zeros on `/joy` disconnect and on upstream command staleness.** v1 had no such safety net.

[Unreleased]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/compare/v0.0.1...HEAD
[0.0.1]: https://github.com/MITRacecarNeo/racecar_neo_ros2_driver/releases/tag/v0.0.1
