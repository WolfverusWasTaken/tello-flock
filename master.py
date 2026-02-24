#!/usr/bin/env python3
"""
Tello 3-drone master controller (1 master + 2 slaves) over Ethernet UDP sync.

- Master sends segment commands to slaves and waits for ACKs.
- Emergency key ('x') broadcasts KILL to slaves AND kills master immediately.
- Land key ('l') broadcasts LAND to slaves and waits for ACKs, then lands master.
"""

import socket, threading, time, sys, termios, tty, select
from djitellopy import Tello

# ================== USER CONFIG ==================
MASTER_IP = "192.168.50.1"          # ethernet IP of THIS master Nano
PORT      = 5005
SLAVES    = [("192.168.50.2", PORT), ("192.168.50.3", PORT)]  # slave ethernet IPs

# Flight tuning (cm, cm/s)
TAKEOFF_UP_CM    = 120
V_CM_S           = 20
HZ               = 20
STRAIGHT_CM      = 300
YAW_DEG_S        = 70   # degrees/second for in-place 180° turn

# ACK timeouts (seconds)
ACK_TIMEOUT_TAKEOFF = 30
ACK_TIMEOUT_LAND    = 20

LOGFILE = "master_log.txt"
# =================================================

def log(msg: str):
    ts = time.strftime("%H:%M:%S")
    with open(LOGFILE, "a") as f:
        f.write(f"[{ts}] {msg}\n")

# ---------------- Non-blocking key reader ----------------
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

# ---------------- UDP master with ACK tracking ----------------
class UdpMaster:
    def __init__(self, bind_ip: str, bind_port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to all interfaces for reliability (ethernet + any others)
        # Still *send* to slave ethernet IPs.
        self.sock.bind(("0.0.0.0", bind_port))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.pending = {}  # msg_id -> set(ips still awaited)

        self.expected_slave_ips = {ip for ip, _ in SLAVES}
        self.master_eth_ip = bind_ip

    def _msg_id(self) -> str:
        return str(int(time.time() * 1000))

    def _handle_incoming(self, data: bytes, addr):
        txt = data.decode(errors="ignore").strip()
        parts = txt.split("|")

        # ACK|<msg_id>|<name>
        if len(parts) >= 2 and parts[0] == "ACK":
            msg_id = parts[1]
            name = parts[2] if len(parts) > 2 else addr[0]
            ip = addr[0]
            with self.lock:
                if msg_id in self.pending and ip in self.pending[msg_id]:
                    self.pending[msg_id].discard(ip)
                    remaining = len(self.pending[msg_id])
                    log(f"ACK from {name} ({ip}) id={msg_id} | remaining={remaining}")
                    print(f"[MASTER] ACK from {name} | remaining: {remaining}")

        # Optional: allow other nodes to signal emergency to master:
        # EMERGENCY|<id> or KILL|<id>
        elif len(parts) == 2 and parts[0] in ("EMERGENCY", "KILL"):
            cmd, msg_id = parts
            ip = addr[0]
            log(f"Received {cmd} trigger from {ip} id={msg_id}")
            print(f"[MASTER] !!! {cmd} trigger received from {ip} !!!")
            # handled by caller via poll_incoming()

    def poll_incoming(self):
        """Non-blocking poll for any incoming packets (ACKs or emergency triggers)."""
        try:
            data, addr = self.sock.recvfrom(1024)
        except socket.timeout:
            return None, None
        self._handle_incoming(data, addr)
        txt = data.decode(errors="ignore").strip()
        parts = txt.split("|")
        if len(parts) == 2 and parts[0] in ("EMERGENCY", "KILL"):
            return parts[0], parts[1]
        return None, None

    def send_and_wait_ack(self, cmd: str, targets, timeout: float, send_count: int = 3) -> bool:
        """
        Send cmd to all targets (send_count times for UDP reliability) and block
        until every target ACKs or timeout expires. Returns True on full ACK.
        """
        msg_id = self._msg_id()
        payload = f"{cmd}|{msg_id}".encode()
        target_ips = [ip for ip, _ in targets]

        with self.lock:
            self.pending[msg_id] = set(target_ips)

        log(f">>> {cmd} id={msg_id} -> {target_ips} timeout={timeout}s")
        print(f"[MASTER] >>> {cmd} -> {target_ips} (timeout {timeout}s)")

        # Burst-send for UDP reliability
        for i in range(send_count):
            for ip, port in targets:
                self.sock.sendto(payload, (ip, port))
            if i < send_count - 1:
                time.sleep(0.05)

        t0 = time.time()
        while time.time() - t0 < timeout:
            # Also allow emergency triggers to interrupt waits
            trig_cmd, trig_id = self.poll_incoming()
            if trig_cmd in ("EMERGENCY", "KILL"):
                # Caller will handle; treat as failure to unblock safely
                log(f"Emergency trigger while waiting ACKs: {trig_cmd} id={trig_id}")
                return False

            with self.lock:
                if not self.pending.get(msg_id, set()):
                    self.pending.pop(msg_id, None)
                    elapsed = time.time() - t0
                    log(f"{cmd} ALL ACKs in {elapsed:.1f}s")
                    print(f"[MASTER] {cmd} all ACKs in {elapsed:.1f}s")
                    return True

        with self.lock:
            missing = sorted(list(self.pending.get(msg_id, set())))
            self.pending.pop(msg_id, None)
        log(f"!!! ACK TIMEOUT cmd={cmd} missing={missing}")
        print(f"[MASTER] !!! ACK TIMEOUT cmd={cmd} missing={missing}")
        return False

    def broadcast(self, cmd: str, targets, send_count: int = 3):
        """Fire-and-forget broadcast (no ACK wait)."""
        msg_id = self._msg_id()
        payload = f"{cmd}|{msg_id}".encode()
        for _ in range(send_count):
            for ip, port in targets:
                self.sock.sendto(payload, (ip, port))
            time.sleep(0.03)

# ---------------- Smooth RC motion helpers ----------------
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

def turn_180(tello: Tello, yaw_deg_s=70, cw=True, hz=20):
    yaw_cmd = int(clamp(yaw_deg_s, 5, 100))
    yaw = yaw_cmd if cw else -yaw_cmd
    duration = 180.0 / yaw_deg_s
    rc_hold(tello, lr=0, fb=0, ud=0, yaw=yaw, duration_s=duration, hz=hz)

# ---------------- Safety actions ----------------
def kill_all(comm: UdpMaster, tello: Tello):
    log("KILL_ALL: broadcasting KILL to slaves")
    print("[MASTER] !!! KILL_ALL: broadcasting KILL to slaves !!!")
    comm.broadcast("KILL", SLAVES)
    try:
        tello.emergency()
    except Exception as e:
        log(f"master emergency error: {e}")
        print(f"[MASTER] emergency error: {e}")

def land_all(comm: UdpMaster, tello: Tello):
    log("LAND_ALL: sending LAND to slaves")
    print("[MASTER] Sending LAND to all slaves...")
    comm.send_and_wait_ack("LAND", SLAVES, timeout=ACK_TIMEOUT_LAND)
    log("LAND_ALL: landing master")
    print("[MASTER] Landing master...")
    try:
        tello.land()
    except Exception as e:
        log(f"master land error: {e}")
        print(f"[MASTER] land error: {e}")

# ---------------- Main ----------------
def main():
    comm = UdpMaster(MASTER_IP, PORT)

    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    log(f"Connected. Battery={bat}%")
    print(f"[MASTER] Connected. Battery={bat}%")

    stop_flag = {"stop": False}

    def key_thread():
        print("\nControls: [x]=KILL(emergency)  [l]=LAND(all)  [q]=quit\n")
        with KeyReader() as kr:
            while not stop_flag["stop"]:
                k = kr.get_key()
                if not k:
                    time.sleep(0.02)
                    continue
                k = k.lower()
                if k == "x":
                    log("!!! EMERGENCY KEY PRESSED !!!")
                    kill_all(comm, tello)
                elif k == "l":
                    log("Manual LAND(all) requested")
                    land_all(comm, tello)
                elif k == "q":
                    stop_flag["stop"] = True
                    break

    threading.Thread(target=key_thread, daemon=True).start()

    input("[MASTER] Press ENTER to start: takeoff -> 1 lap (sync) -> land...")

    # STEP 1 — Master takeoff & climb
    log("STEP 1: master takeoff+climb")
    print("[MASTER] STEP 1: Taking off...")
    tello.takeoff()
    if TAKEOFF_UP_CM > 0:
        tello.move_up(TAKEOFF_UP_CM)
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    print("[MASTER] Takeoff complete.")

    # STEP 2 — Slave takeoff
    log("STEP 2: command slaves TAKEOFF")
    print("[MASTER] STEP 2: Commanding slaves TAKEOFF...")
    if not comm.send_and_wait_ack("TAKEOFF", SLAVES, timeout=ACK_TIMEOUT_TAKEOFF):
        print("[MASTER] TAKEOFF ACK failed -> LAND_ALL")
        land_all(comm, tello)
        return

    # STEP 3 — Master flies autonomously (slaves do the same in parallel)
    log("STEP 3: master forward")
    print("[MASTER] STEP 3: Moving forward...")
    smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
    tello.send_rc_control(0, 0, 0, 0)

    # STEP 4 — 180° turn
    log("STEP 4: master 180 turn")
    print("[MASTER] STEP 4: 180 turn...")
    turn_180(tello, YAW_DEG_S, cw=True, hz=HZ)
    tello.send_rc_control(0, 0, 0, 0)

    # STEP 5 — Return straight
    log("STEP 5: master return")
    print("[MASTER] STEP 5: Returning...")
    smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
    tello.send_rc_control(0, 0, 0, 0)

    # STEP 6 — Land all
    log("STEP 6: land all")
    land_all(comm, tello)
    stop_flag["stop"] = True
    print("[MASTER] Done.")

if __name__ == "__main__":
    main()
