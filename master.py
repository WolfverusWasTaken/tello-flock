#!/usr/bin/env python3
import socket, threading, time, sys, termios, tty, select, math
from djitellopy import Tello

def log(msg):
    with open('master_log.txt', 'a') as f:
        f.write('[INFO] ' + str(msg) + '\n')

# =============== ETHERNET SYNC CONFIG ===============
MASTER_IP = "192.168.50.1"
PORT = 5005
SLAVES = [("192.168.50.2", PORT), ("192.168.50.3", PORT)]

ACK_TIMEOUT = 5.0
ACK_RETRIES = 3

# =============== FLIGHT CONFIG (TUNE) ===============
TAKEOFF_UP_CM = 120      # climb after takeoff
V_CM_S = 20              # forward speed during smooth segments (start slow)
HZ = 20                  # RC update rate (Hz)
STRAIGHT_CM = 300        # 3m straights

# Smaller-radius track + 0.3m lateral gap by different radii
RADIUS_CM_MASTER = 120   # centerline radius (cm). tighter than 150.
# (slaves have their own radii in their scripts)

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
        self.pending = {}  # msg_id -> set(ips)

    def _msg_id(self):
        return str(int(time.time() * 1000))

    def _handle_incoming(self, data, addr):
        txt = data.decode(errors="ignore").strip()
        parts = txt.split("|")
        # ACK|<msg_id>|<name>
        if len(parts) >= 2 and parts[0] == "ACK":
            msg_id = parts[1]
            ip = addr[0]
            with self.lock:
                if msg_id in self.pending:
                    self.pending[msg_id].discard(ip)

    def send_and_wait_ack(self, cmd: str, targets):
        msg_id = self._msg_id()
        payload = f"{cmd}|{msg_id}".encode()
        with self.lock:
            self.pending[msg_id] = set(ip for ip, _ in targets)

        for _ in range(ACK_RETRIES):
            for ip, port in targets:
                self.sock.sendto(payload, (ip, port))

            t0 = time.time()
            while time.time() - t0 < ACK_TIMEOUT:
                with self.lock:
                    if not self.pending.get(msg_id, set()):
                        self.pending.pop(msg_id, None)
                        return True
                try:
                    data, addr = self.sock.recvfrom(1024)
                except socket.timeout:
                    continue
                self._handle_incoming(data, addr)

        with self.lock:
            missing = sorted(list(self.pending.get(msg_id, set())))
        print(f"[MASTER] ACK FAILED cmd={cmd} missing={missing}")
        return False

    def broadcast(self, cmd: str, targets):
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

    yaw_deg_s = (v / R) * (180.0 / math.pi)  # ideal deg/s
    yaw_cmd = int(round(clamp(yaw_deg_s, 5, 100)))  # Tello range-ish
    yaw = yaw_cmd if cw else -yaw_cmd

    duration = (math.pi * R) / v  # semicircle time
    rc_hold(tello, lr=0, fb=int(round(v)), ud=0, yaw=yaw, duration_s=duration, hz=hz)

def do_one_lap(tello: Tello, radius_cm, straight_cm=300, v_cm_s=20, hz=20):
    smooth_straight(tello, straight_cm, v_cm_s, hz)
    smooth_semicircle(tello, radius_cm, v_cm_s, cw=True, hz=hz)
    smooth_straight(tello, straight_cm, v_cm_s, hz)
    smooth_semicircle(tello, radius_cm, v_cm_s, cw=True, hz=hz)

# =============== MAIN ===============
def main():
    comm = UdpMaster(MASTER_IP, PORT)

    tello = Tello()
    tello.connect()
    log(f"MASTER Connected to Tello (wifi 192.168.10.1). Battery={tello.get_battery()}%")
    print(f"[MASTER] Connected to Tello (wifi 192.168.10.1). Battery={tello.get_battery()}%")

    stop_flag = {"stop": False}

    def emergency_kill():
        print("\nControls: [x]=EMERGENCY(KILL)  [l]=LAND(all)  [q]=quit\n")
        with KeyReader() as kr:
            while not stop_flag["stop"]:
                k = kr.get_key()
                if not k:
                    time.sleep(0.02)
                    continue
                k = k.lower()
                if k == "x":
                    log("MASTER !!! EMERGENCY !!!")
                    print("[MASTER] !!! EMERGENCY !!!")
                    comm.broadcast("EMERGENCY", SLAVES)
                    try:
                        tello.emergency()
                    except Exception as e:
                        log(f"MASTER local emergency error: {e}")
                        print("[MASTER] local emergency error:", e)
                elif k == "l":
                    log("MASTER LAND(all)")
                    print("[MASTER] LAND(all)")
                    comm.send_and_wait_ack("LAND", SLAVES)
                    try:
                        tello.land()
                    except Exception as e:
                        log(f"MASTER local land error: {e}")
                        print("[MASTER] local land error:", e)
                elif k == "q":
                    stop_flag["stop"] = True
                    break

    threading.Thread(target=emergency_kill, daemon=True).start()

    input("[MASTER] Press ENTER to start: MASTER takeoff -> SLAVES takeoff -> 1 lap -> land...")

    # 1) MASTER takeoff first
    log("MASTER Taking off...")
    print("[MASTER] Taking off...")
    tello.takeoff()
    if TAKEOFF_UP_CM > 0:
        tello.move_up(TAKEOFF_UP_CM)
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)

    # 2) Ask SLAVES to takeoff, wait for TAKEOFF COMPLETE ACKs
    log("MASTER Commanding slaves TAKEOFF, waiting for completion...")
    print("[MASTER] Commanding slaves TAKEOFF, waiting for completion...")
    if not comm.send_and_wait_ack("TAKEOFF", SLAVES):
        log("MASTER Slave takeoff failed -> LAND(all)")
        print("[MASTER] Slave takeoff failed -> LAND(all)")
        comm.send_and_wait_ack("LAND", SLAVES)
        try:
            tello.land()
        except Exception as e:
            log(f"MASTER land error: {e}")
            pass
        return

    # 3) Start lap together
    log("MASTER Slaves confirmed airborne. Starting lap...")
    print("[MASTER] Slaves confirmed airborne. Starting lap...")
    if not comm.send_and_wait_ack("RUN_LAP", SLAVES):
        log("MASTER RUN_LAP sync failed -> EMERGENCY")
        print("[MASTER] RUN_LAP sync failed -> EMERGENCY")
        comm.broadcast("EMERGENCY", SLAVES)
        try:
            tello.emergency()
        except Exception as e:
            log(f"MASTER emergency error: {e}")
            pass
        return

    log("MASTER Running lap...")
    do_one_lap(tello, RADIUS_CM_MASTER, STRAIGHT_CM, V_CM_S, HZ)

    # 4) Land together
    log("MASTER Lap done. LAND(all)")
    print("[MASTER] Lap done. LAND(all)")
    comm.send_and_wait_ack("LAND", SLAVES)
    tello.land()

    stop_flag["stop"] = True
    log("MASTER Done.")
    print("[MASTER] Done.")

if __name__ == "__main__":
    main()