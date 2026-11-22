# so101

**An SO-101 leader and follower arm, run from a Mac with LeRobot.**
This repo is the working folder for the pair: getting both arms calibrated,
driving the follower from the leader, and the first programs that move the
follower on their own.

It sits on top of [LeRobot](https://github.com/huggingface/lerobot) and uses
its motor bus, arm classes and calibration files unchanged.

This is an early project. What works today: both arms calibrated,
teleoperation at 60 Hz, and a first motion script for the follower. There are
no cameras on the rig yet, so there is no dataset recording or policy training
here.

- [What it does](#what-it-does)
- [Hardware](#hardware)
- [Quick start](#quick-start)
- [How calibration works](#how-calibration-works)
- [Repository guide](#repository-guide)
- [Credits](#credits)

## What it does

- **Teleoperates.** The follower copies the leader through
  `lerobot-teleoperate`, started and stopped from a local page. Stopping sends
  SIGINT so LeRobot releases the follower's torque cleanly.
- **Moves on its own.** `wave.py` reads the follower's pose and eases each
  joint 10° out and back, with a cap on how far one command can move a joint.
- **Calibrates from the browser.** `calibrate_ui.py` serves a page that draws
  the arm in the pose you need to hold, tracks every joint live while you
  sweep it, and will not save a calibration that is not a real sweep. See
  [How calibration works](#how-calibration-works).

## Hardware

| Part | Notes |
|---|---|
| Follower arm | SO-101, six Feetech STS3215 bus servos, on its own USB serial port. |
| Leader arm | SO-101 leader, same servo bus, on a second USB serial port. |
| Computer | Developed on an Apple silicon Mac. The serial port names in the code are macOS `/dev/cu.usbmodem…` paths. |
| Cameras | None yet. |

## Quick start

```bash
uv venv --python 3.12 .venv
VIRTUAL_ENV=$PWD/.venv uv pip install 'lerobot[feetech]'
```

Developed against `lerobot` 0.7.0. Then set your own serial ports: edit `PORTS`
at the top of `calibrate_ui.py` and `PORT` in `wave.py`.
`.venv/bin/lerobot-find-port` tells you which port is which arm.

```bash
.venv/bin/python calibrate_ui.py     # opens http://127.0.0.1:8101
```

Calibrate the follower, then the leader, then press Teleoperate. Pass
`--no-open` to keep it from opening a browser tab.

```bash
.venv/bin/python wave.py             # follower only, after calibrating
```

`wave.py` releases torque when it exits, so the arm goes limp. Keep a hand near
it.

## How calibration works

The page loads the SO-101 URDF with three.js, so you match a picture of the
pose rather than read a description of it.

1. **Middle.** With torque off, you hold the arm like the model on screen and
   press the button. The server sets half-turn homing offsets from that pose.
2. **Sweep.** You move each joint through its full range, one at a time. The
   server reads positions about 30 times a second and keeps the minimum and
   maximum for each joint. `wrist_roll` spans the whole encoder, so it is not
   swept and gets the full range.
3. **Save.** A joint that covered fewer than 300 encoder steps was not swept,
   and one that covered more than 3900 means the encoder wrapped, so the
   middle pose was off. Either way the page names the joints and nothing is
   written. Otherwise the calibration goes to the servos and to LeRobot's
   calibration file for that arm, the same one `lerobot-calibrate` writes.

The server owns the serial ports. One worker thread per session does all the
bus I/O, and the page only polls `/state` and posts button presses. A cable
pulled mid-run shows up as an error screen in the page, not a hung terminal.

A servo turned across its encoder zero while powered reports one turn too many
(6200, say) until it is reconnected. Readings are folded back into one turn
before they are used.

## Repository guide

| Path | What it is |
|---|---|
| `calibrate_ui.py` | The local server: calibration sessions, teleoperation process control, and the HTTP endpoints the page uses. Standard library plus `lerobot`. |
| `ui.html` | The page: home, middle pose, sweep, done and error screens, with the 3D arm. Loads three.js and urdf-loader from a CDN, so it needs internet. |
| `wave.py` | First program for the follower: print the pose, then wiggle each joint. |
| `model/` | The SO-101 URDF and its STL meshes, used by the page. |

## Credits

- [LeRobot](https://github.com/huggingface/lerobot) by Hugging Face, for the
  motor bus, the arm classes and teleoperation.
- [SO-ARM100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100) by
  TheRobotStudio, for the arm. The URDF in `model/` was generated with
  onshape-to-robot.
