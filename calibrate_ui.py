"""Browser-guided calibration and teleoperation for the SO-101 leader + follower.

    .venv/bin/python calibrate_ui.py      # then follow http://127.0.0.1:8101

The page (ui.html) shows how to pose the arm and tracks each joint live. This server owns
the serial ports: one worker thread per calibration session does all the bus I/O.
"""

import json
import queue
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

HERE = Path(__file__).parent
HTTP_PORT = 8101
PORTS = {"follower": "/dev/cu.usbmodem5AAF2199921", "leader": "/dev/cu.usbmodem5AAF2181311"}
ARMS = {
    "follower": lambda: SO101Follower(SO101FollowerConfig(port=PORTS["follower"], id="follower")),
    "leader": lambda: SO101Leader(SO101LeaderConfig(port=PORTS["leader"], id="leader")),
}
CALIB_FILES = {name: make().calibration_fpath for name, make in ARMS.items()}
FULL_TURN = ["wrist_roll"]  # spans the whole encoder, so it is not swept
MIN_RANGE = 300  # encoder steps a swept joint must cover (4096 per turn)
MAX_RANGE = 3900  # more than this means the encoder wrapped: the middle pose was off

lock = threading.Lock()
state = {"phase": "idle", "arm": None, "joints": {}, "error": None}
commands: queue.Queue[str] = queue.Queue()
teleop = {"proc": None, "error": None}


def set_state(**kw) -> None:
    with lock:
        state.update(kw)


def session(name: str) -> None:
    """One calibration run: connect, wait for the middle pose, record ranges, save."""
    arm = ARMS[name]()
    bus = arm.bus
    phase, homing, mins, maxes = "middle", {}, {}, {}
    try:
        bus.connect()
        bus.disable_torque()
        bus.set_operating_mode("position")
        set_state(phase=phase)
        while True:
            try:
                cmd = commands.get_nowait()
            except queue.Empty:
                cmd = None
            if cmd == "cancel":
                set_state(phase="idle", arm=None, joints={})
                return
            if cmd == "redo":
                phase, mins, maxes = "middle", {}, {}
                set_state(phase=phase, error=None)
            if cmd == "middle" and phase == "middle":
                homing = bus.set_half_turn_homings()
                mins = bus.sync_read("Present_Position", normalize=False, num_retry=5)
                mins = {m: p % 4096 for m, p in mins.items()}
                maxes = dict(mins)
                phase = "sweep"
                set_state(phase=phase, error=None)
            if cmd == "save" and phase == "sweep":
                swept = [m for m in bus.motors if m not in FULL_TURN]
                bad = [m for m in swept if not MIN_RANGE <= maxes[m] - mins[m] <= MAX_RANGE]
                if bad:
                    set_state(error="These joints need a proper sweep (or the middle pose was off): " + ", ".join(bad))
                else:
                    for m in FULL_TURN:
                        mins[m], maxes[m] = 0, bus.resolution(bus.motors[m].model) - 1
                    arm.calibration = {
                        m: MotorCalibration(
                            id=bus.motors[m].id,
                            drive_mode=0,
                            homing_offset=int(homing[m]),
                            range_min=int(mins[m]),
                            range_max=int(maxes[m]),
                        )
                        for m in bus.motors
                    }
                    bus.write_calibration(arm.calibration)
                    arm._save_calibration()
                    set_state(phase="done", error=None)
                    return

            pos = bus.sync_read("Present_Position", normalize=False, num_retry=5)
            # A servo turned across its encoder zero while powered reports one turn too many
            # (e.g. 6200) until it is reconnected. Fold that back into one turn.
            pos = {m: p % 4096 for m, p in pos.items()}
            if phase == "sweep":
                mins = {m: min(pos[m], mins[m]) for m in pos}
                maxes = {m: max(pos[m], maxes[m]) for m in pos}
            joints = {m: {"pos": int(pos[m]), "min": int(mins.get(m, pos[m])), "max": int(maxes.get(m, pos[m]))} for m in pos}
            set_state(joints=joints)
            time.sleep(0.03)
    except Exception as e:  # shown in the page: a cable pulled mid-run lands here
        set_state(phase="error", error=f"{type(e).__name__}: {e}".strip())
    finally:
        try:
            bus.disconnect()
        except Exception:
            pass


def teleop_running() -> bool:
    proc = teleop["proc"]
    if proc is None:
        return False
    if proc.poll() is None:
        return True
    if proc.returncode not in (0, -signal.SIGINT, 130):
        teleop["error"] = "Teleoperation stopped: " + (HERE / "teleop.log").read_text()[-400:]
    teleop["proc"] = None
    return False


def teleop_start() -> None:
    teleop["error"] = None
    cmd = [
        str(Path(sys.executable).parent / "lerobot-teleoperate"),
        "--robot.type=so101_follower", f"--robot.port={PORTS['follower']}", "--robot.id=follower",
        "--teleop.type=so101_leader", f"--teleop.port={PORTS['leader']}", "--teleop.id=leader",
    ]  # fmt: skip
    log = open(HERE / "teleop.log", "w")
    teleop["proc"] = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)


def teleop_stop() -> None:
    proc = teleop["proc"]
    if proc and proc.poll() is None:
        proc.send_signal(signal.SIGINT)  # lets lerobot release the follower's torque
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args) -> None:
        pass

    def reply(self, body: bytes, ctype: str = "application/json", code: int = 200) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/":
            self.reply((HERE / "ui.html").read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/state":
            running = teleop_running()
            with lock:
                body = dict(state)
            body["calibrated"] = {name: path.exists() for name, path in CALIB_FILES.items()}
            body["teleop"] = {"running": running, "error": teleop["error"]}
            self.reply(json.dumps(body).encode())
        elif self.path.startswith("/model/"):
            file = (HERE / self.path.lstrip("/")).resolve()
            if file.is_file() and file.is_relative_to(HERE / "model"):
                self.reply(file.read_bytes(), "application/octet-stream")
            else:
                self.reply(b"{}", code=404)
        else:
            self.reply(b"{}", code=404)

    def do_POST(self) -> None:
        action = self.path.strip("/")
        with lock:
            phase = state["phase"]
        busy = phase in ("connecting", "middle", "sweep")
        if action in ("start/follower", "start/leader") and not busy and not teleop_running():
            while not commands.empty():
                commands.get_nowait()
            name = action.split("/")[1]
            set_state(phase="connecting", arm=name, joints={}, error=None)
            threading.Thread(target=session, args=(name,), daemon=True).start()
        elif action in ("middle", "save", "redo", "cancel") and busy:
            commands.put(action)
        elif action == "home" and not busy:
            set_state(phase="idle", arm=None, joints={}, error=None)
        elif action == "teleop/start" and not busy and not teleop_running():
            teleop_start()
        elif action == "teleop/stop":
            teleop_stop()
        self.reply(b"{}")


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    url = f"http://127.0.0.1:{HTTP_PORT}"
    print(f"SO-101 setup running at {url}  (Ctrl+C to quit)", flush=True)
    if "--no-open" not in sys.argv:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_stop()
