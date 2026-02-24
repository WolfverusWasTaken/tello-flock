#!/usr/bin/env python3
import socket, threading, time, sys, termios, tty, select, math
from djitellopy import Tello

def log(msg):
    ts = time.strftime("%H:%M:%S")
    with open('master_log.txt', 'a') as f:
        f.write(f"[{ts}] {msg}\n")

# =============== ETHERNET SYNC CONFIG ===============
MASTER_IP = "192.168.50.1"
PORT = 5005
SLAVES = [("192.168.50.2", PORT), ("192.168.50.3", PORT)]

# Per-command ACK timeouts (seconds).
# Each timeout must exceed the slowest slave's execution time for that segment.
ACK_TIMEOUT_TAKEOFF = 30   # takeoff + climb ~15s
ACK_TIMEOUT_MOVE    = 25   # 300cm / 20cm/s = 15s + buffer
ACK_TIMEOUT_TURN    = 35   # largest slave (R=150cm): π*150/20 ≈ 24s + buffer
ACK_TIMEOUT_LAND    = 20   # landing ~5-10s

# =============== FLIGHT CONFIG (TUNE) ===============
TAKEOFF_UP_CM    = 120
V_CM_S           = 20
HZ               = 20
STRAIGHT_CM      = 300
RADIUS_CM_MASTER = 120

# =============== NON-BLOCKING KEY READER ===============
class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
    def __enter__(self):
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def get_key(self):
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None

# =============== UDP MASTER ===============
class UdpMaster:
    def __init__(self, bind_ip, bind_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, bind_port))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.pending = {}  # msg_id -> set(ips still awaited)

    def _msg_id(self):
        return str(int(time.time() * 1000))

    def _handle_incoming(self, data, addr):
        txt = data.decode(errors="ignore").strip()
        parts = txt.split("|")
        # Expected: ACK|<msg_id>|<name>
        if len(parts) >= 2 and parts[0] == "ACK":
            msg_id = parts[1]
            name = parts[2] if len(parts) > 2 else addr[0]
            ip = addr[0]
            with self.lock:
                if msg_id in self.pending and ip in self.pending[msg_id]:
                    self.pending[msg_id].discard(ip)
                    remaining = len(self.pending[msg_id])
                    log(f"MASTER ACK from {name} ({ip}) cmd_id={msg_id} | still waiting: {remaining}")
                    print(f"[MASTER] ACK from {name} | still waiting: {remaining}")

    def send_and_wait_ack(self, cmd: str, targets, timeout: float, send_count: int = 3) -> bool:
        """
        Send cmd to all targets (send_count times for UDP reliability) and block
        until every target ACKs or timeout expires.  Returns True on full ACK.
        """
        msg_id = self._msg_id()
        payload = f"{cmd}|{msg_id}".encode()
        target_ips = [ip for ip, _ in targets]

        with self.lock:
            self.pending[msg_id] = set(target_ips)

        log(f"MASTER >>> {cmd} (id={msg_id}) -> {target_ips}  timeout={timeout}s")
        print(f"[MASTER] >>> {cmd} -> {target_ips}  (timeout {timeout}s)")

        # Burst-send for UDP reliability
        for i in range(send_count):
            for ip, port in targets:
                self.sock.sendto(payload, (ip, port))
            if i < send_count - 1:
                time.sleep(0.05)

        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if not self.pending.get(msg_id, set()):
                    self.pending.pop(msg_id, None)
                    elapsed = time.time() - t0
                    log(f"MASTER {cmd} ALL ACKs received in {elapsed:.1f}s")
                    print(f"[MASTER] {cmd} all ACKs received in {elapsed:.1f}s")
                    return True
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            self._handle_incoming(data, addr)

        with self.lock:
            missing = sorted(list(self.pending.get(msg_id, set())))
            self.pending.pop(msg_id, None)
        log(f"MASTER !!! ACK TIMEOUT cmd={cmd} missing={missing}")
        print(f"[MASTER] !!! ACK TIMEOUT cmd={cmd} missing={missing}")
        return False

    def broadcast(self, cmd: str, targets):
        """Fire-and-forget broadcast (no ACK wait), used for EMERGENCY."""
        msg_id = self._msg_id()
        payload = f"{cmd}|{msg_id}".encode()
        for ip, port in targets:
            self.sock.sendto(payload, (ip, port))
            self.sock.sendto(payload, (ip, port))

# =============== SMOOTH RC MOTION ===============
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def rc_hold(tello: Tello, lr, fb, ud, yaw, duration_s, hz=20):
    period = 1.0 / hz
    t_end = time.time() + duration_s
    while time.time() < t_end:
        tello.send_rc_control(lr, fb, ud, yaw)
        time.sleep(period)
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.15)

def smooth_straight(tello: Tello, distance_cm, v_cm_s=20, hz=20):
    v = int(clamp(v_cm_s, 10, 60))
    duration = float(distance_cm) / float(v)
    rc_hold(tello, lr=0, fb=v, ud=0, yaw=0, duration_s=duration, hz=hz)

def smooth_semicircle(tello: Tello, radius_cm, v_cm_s=20, cw=True, hz=20):
    v = float(clamp(v_cm_s, 10, 60))
    R = float(radius_cm)
    yaw_deg_s = (v / R) * (180.0 / math.pi)
    yaw_cmd = int(round(clamp(yaw_deg_s, 5, 100)))
    yaw = yaw_cmd if cw else -yaw_cmd
    duration = (math.pi * R) / v
    rc_hold(tello, lr=0, fb=int(round(v)), ud=0, yaw=yaw, duration_s=duration, hz=hz)

# =============== HELPERS ===============
def land_all(comm: UdpMaster, tello: Tello):
    log("MASTER land_all: sending LAND to slaves")
    print("[MASTER] Sending LAND to all slaves...")
    comm.send_and_wait_ack("LAND", SLAVES, timeout=ACK_TIMEOUT_LAND)
    log("MASTER land_all: landing master")
    print("[MASTER] Landing master...")
    try:
        tello.land()
    except Exception as e:
        log(f"MASTER land error: {e}")
        print(f"[MASTER] land error: {e}")

def abort(comm: UdpMaster, tello: Tello, stop_flag: dict, reason: str):
    log(f"MASTER ABORT: {reason}")
    print(f"[MASTER] ABORT: {reason}")
    land_all(comm, tello)
    stop_flag["stop"] = True

# =============== MAIN ===============
def main():
    comm = UdpMaster(MASTER_IP, PORT)

    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    log(f"MASTER Connected. Battery={bat}%")
    print(f"[MASTER] Connected. Battery={bat}%")

    stop_flag = {"stop": False}

    def emergency_kill():
        print("\nControls: [x]=EMERGENCY(kill motors)  [l]=LAND(all)  [q]=quit\n")
        with KeyReader() as kr:
            while not stop_flag["stop"]:
                k = kr.get_key()
                if not k:
                    time.sleep(0.02)
                    continue
                k = k.lower()
                if k == "x":
                    log("MASTER !!! EMERGENCY KEY PRESSED !!!")
                    print("[MASTER] !!! EMERGENCY !!!")
                    comm.broadcast("EMERGENCY", SLAVES)
                    try:
                        tello.emergency()
                    except Exception as e:
                        log(f"MASTER emergency error: {e}")
                elif k == "l":
                    log("MASTER manual LAND(all) requested")
                    print("[MASTER] LAND(all) requested")
                    land_all(comm, tello)
                elif k == "q":
                    stop_flag["stop"] = True
                    break

    threading.Thread(target=emergency_kill, daemon=True).start()

    input("[MASTER] Press ENTER to start: takeoff -> 1 lap (per-segment sync) -> land...")

    # ═══════════════════════════════════════════════════════════
    # STEP 1 — MASTER TAKEOFF & CLIMB
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 1: Takeoff & climb ──")
    print("[MASTER] STEP 1: Taking off...")
    tello.takeoff()
    if TAKEOFF_UP_CM > 0:
        tello.move_up(TAKEOFF_UP_CM)
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    log("MASTER takeoff+climb complete")
    print("[MASTER] Takeoff complete.")

    # ═══════════════════════════════════════════════════════════
    # STEP 2 — COMMAND SLAVES: TAKEOFF  (wait for TAKEOFF DONE ACKs)
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 2: Commanding slaves TAKEOFF ──")
    print("[MASTER] STEP 2: Commanding slaves TAKEOFF...")
    if not comm.send_and_wait_ack("TAKEOFF", SLAVES, timeout=ACK_TIMEOUT_TAKEOFF):
        abort(comm, tello, stop_flag, "slaves did not confirm TAKEOFF")
        return
    log("MASTER slaves TAKEOFF DONE confirmed")
    print("[MASTER] All slaves airborne. Starting lap segments.")

    # ═══════════════════════════════════════════════════════════
    # STEP 3a — MASTER: straight segment 1
    # STEP 3b — COMMAND SLAVES: MOVE_FORWARD  (wait for DONE ACKs)
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 3a: Moving forward (straight 1) ──")
    print("[MASTER] STEP 3a: Moving forward (straight 1)...")
    smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
    tello.send_rc_control(0, 0, 0, 0)
    log("MASTER straight 1 done. Pausing 1s before commanding slaves.")
    print("[MASTER] Straight 1 done. Waiting 1s...")
    time.sleep(1.0)

    log("MASTER ── STEP 3b: Commanding slaves MOVE_FORWARD ──")
    print("[MASTER] STEP 3b: Commanding slaves MOVE_FORWARD...")
    if not comm.send_and_wait_ack("MOVE_FORWARD", SLAVES, timeout=ACK_TIMEOUT_MOVE):
        abort(comm, tello, stop_flag, "slaves did not confirm MOVE_FORWARD (seg 1)")
        return
    log("MASTER slaves MOVE_FORWARD (seg 1) DONE confirmed")
    print("[MASTER] All slaves MOVE_FORWARD (seg 1) done.")

    # ═══════════════════════════════════════════════════════════
    # STEP 4a — MASTER: semicircle / turn 1
    # STEP 4b — COMMAND SLAVES: LAP  (wait for DONE ACKs)
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 4a: Turning (semicircle 1) ──")
    print("[MASTER] STEP 4a: Turning (semicircle 1)...")
    smooth_semicircle(tello, RADIUS_CM_MASTER, V_CM_S, cw=True, hz=HZ)
    tello.send_rc_control(0, 0, 0, 0)
    log("MASTER turn 1 done. Pausing 1s before commanding slaves.")
    print("[MASTER] Turn 1 done. Waiting 1s...")
    time.sleep(1.0)

    log("MASTER ── STEP 4b: Commanding slaves LAP (turn 1) ──")
    print("[MASTER] STEP 4b: Commanding slaves LAP (turn 1)...")
    if not comm.send_and_wait_ack("LAP", SLAVES, timeout=ACK_TIMEOUT_TURN):
        abort(comm, tello, stop_flag, "slaves did not confirm LAP (turn 1)")
        return
    log("MASTER slaves LAP (turn 1) DONE confirmed")
    print("[MASTER] All slaves LAP (turn 1) done.")

    # ═══════════════════════════════════════════════════════════
    # STEP 5a — MASTER: straight segment 2
    # STEP 5b — COMMAND SLAVES: MOVE_FORWARD  (wait for DONE ACKs)
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 5a: Moving forward (straight 2) ──")
    print("[MASTER] STEP 5a: Moving forward (straight 2)...")
    smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
    tello.send_rc_control(0, 0, 0, 0)
    log("MASTER straight 2 done. Pausing 1s before commanding slaves.")
    print("[MASTER] Straight 2 done. Waiting 1s...")
    time.sleep(1.0)

    log("MASTER ── STEP 5b: Commanding slaves MOVE_FORWARD (seg 2) ──")
    print("[MASTER] STEP 5b: Commanding slaves MOVE_FORWARD (seg 2)...")
    if not comm.send_and_wait_ack("MOVE_FORWARD", SLAVES, timeout=ACK_TIMEOUT_MOVE):
        abort(comm, tello, stop_flag, "slaves did not confirm MOVE_FORWARD (seg 2)")
        return
    log("MASTER slaves MOVE_FORWARD (seg 2) DONE confirmed")
    print("[MASTER] All slaves MOVE_FORWARD (seg 2) done.")

    # ═══════════════════════════════════════════════════════════
    # STEP 6a — MASTER: semicircle / turn 2
    # STEP 6b — COMMAND SLAVES: LAP  (wait for DONE ACKs)
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 6a: Turning (semicircle 2) ──")
    print("[MASTER] STEP 6a: Turning (semicircle 2)...")
    smooth_semicircle(tello, RADIUS_CM_MASTER, V_CM_S, cw=True, hz=HZ)
    tello.send_rc_control(0, 0, 0, 0)
    log("MASTER turn 2 done. Pausing 1s before commanding slaves.")
    print("[MASTER] Turn 2 done. Waiting 1s...")
    time.sleep(1.0)

    log("MASTER ── STEP 6b: Commanding slaves LAP (turn 2) ──")
    print("[MASTER] STEP 6b: Commanding slaves LAP (turn 2)...")
    if not comm.send_and_wait_ack("LAP", SLAVES, timeout=ACK_TIMEOUT_TURN):
        abort(comm, tello, stop_flag, "slaves did not confirm LAP (turn 2)")
        return
    log("MASTER slaves LAP (turn 2) DONE confirmed")
    print("[MASTER] All slaves LAP (turn 2) done.")

    # ═══════════════════════════════════════════════════════════
    # STEP 7 — LAND ALL
    # ═══════════════════════════════════════════════════════════
    log("MASTER ── STEP 7: Lap complete. Landing all drones ──")
    print("[MASTER] STEP 7: Lap complete. Landing all drones...")
    land_all(comm, tello)

    stop_flag["stop"] = True
    log("MASTER sequence complete. All drones landed.")
    print("[MASTER] Done.")

if __name__ == "__main__":
    main()
