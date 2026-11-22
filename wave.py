"""First program for the SO-101 follower: read the pose, then gently wiggle each joint.

Run after calibrating:  .venv/bin/python wave.py
Angles are in degrees (gripper is 0-100 % open). Ctrl+C stops and releases torque.
"""

import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = "/dev/cu.usbmodem5AAF2199921"
ROBOT_ID = "follower"  # must match the --robot.id used for lerobot-calibrate

WIGGLE = 10.0  # degrees (or % for the gripper) each joint moves away from its start
STEP_S = 0.02  # 50 Hz


def move(robot, start: dict, target: dict, seconds: float = 1.0) -> None:
    """Ease from `start` to `target` so no joint jumps."""
    steps = max(1, int(seconds / STEP_S))
    for i in range(1, steps + 1):
        t = i / steps
        s = t * t * (3 - 2 * t)  # smoothstep
        robot.send_action({k: start[k] + (target[k] - start[k]) * s for k in start})
        time.sleep(STEP_S)


def main() -> None:
    # max_relative_target caps how far one command can move a joint, as a safety net.
    robot = SO101Follower(SO101FollowerConfig(port=PORT, id=ROBOT_ID, max_relative_target=10.0))
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        robot.disconnect()
        raise SystemExit("Arm is not calibrated yet. Run the lerobot-calibrate command first.")

    try:
        home = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
        print("Current pose:")
        for k, v in home.items():
            print(f"  {k:20s} {v:7.1f}")

        for joint in home:
            print(f"Wiggling {joint}")
            out = dict(home)
            # Open the gripper if it is closed, close it a little if it is open.
            out[joint] += -WIGGLE if joint == "gripper.pos" and home[joint] > 50 else WIGGLE
            move(robot, home, out)
            move(robot, out, home)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        robot.disconnect()  # releases torque: the arm goes limp, so keep a hand near it


if __name__ == "__main__":
    main()
