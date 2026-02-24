#!/usr/bin/env python3
"""
Tello slave controller (runs on its own Nano connected to its own Tello Wi‑Fi + ethernet).

Listens for UDP commands from master on PORT and executes:
- TAKEOFF
- MOVE_FORWARD
- LAP
- LAND
- KILL / EMERGENCY
Sends ACK|<msg_id>|<NAME> back to master after handling each command.
"""

import socket, threading, time, math
from djitellopy import Tello

# ================== USER CONFIG ==================
SELF_IP   = "192.168.50.2"
MASTER_IP = "192.168.50.1"
PORT      = 5005
NAME      = "SLAVE_1"

# Flight tuning (cm, cm/s)
TAKEOFF_UP_CM = 120
V_CM_S        = 20
HZ            = 20
STRAIGHT_CM   = 300
RADIUS_CM     = 90

LOGFILE = "slave1_log.txt"
# =================================================

def log(msg: str):
    ts = time.strftime("%H:%M:%S")
    with open(LOGFILE, "a") as f:
        f.write(f"[{ts}] {msg}\n")

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

def main():
    tello = Tello()
    tello.connect()
    bat = tello.get_battery()
    log(f"{NAME} Connected. Battery={bat}%")
    print(f"[{NAME}] Connected. Battery={bat}%")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Listen on all interfaces so you don't miss packets if the OS/interface naming changes
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(1.0)
    log(f"{NAME} Listening on 0.0.0.0:{PORT} (ethernet IP {SELF_IP})")
    print(f"[{NAME}] Listening on 0.0.0.0:{PORT} (ethernet IP {SELF_IP})")

    busy = threading.Lock()

    # Deduplication: avoid re-running the same msg_id (UDP duplicates)
    seen_lock     = threading.Lock()
    completed_ids = set()

    def send_ack(msg_id: str):
        pkt = f"ACK|{msg_id}|{NAME}".encode()
        sock.sendto(pkt, (MASTER_IP, PORT))
        log(f"{NAME} ACK sent (id={msg_id})")

    def handle(cmd: str, msg_id: str):
        # If already completed, re-ACK (helps master if it missed our ACK)
        with seen_lock:
            if msg_id in completed_ids:
                log(f"{NAME} Duplicate {cmd} id={msg_id} -> re-ACK")
                send_ack(msg_id)
                return

        # Busy guard: ignore overlapping commands (shouldn't happen if master sync is correct)
        if not busy.acquire(blocking=False):
            log(f"{NAME} Busy -> dropping {cmd} id={msg_id}")
            return

        try:
            if cmd == "TAKEOFF":
                log(f"{NAME} TAKEOFF start")
                print(f"[{NAME}] TAKEOFF...")
                tello.takeoff()
                if TAKEOFF_UP_CM > 0:
                    tello.move_up(TAKEOFF_UP_CM)
                tello.send_rc_control(0, 0, 0, 0)
                time.sleep(0.3)
                print(f"[{NAME}] TAKEOFF DONE -> ACK")
                send_ack(msg_id)

            elif cmd == "MOVE_FORWARD":
                log(f"{NAME} MOVE_FORWARD {STRAIGHT_CM}cm")
                print(f"[{NAME}] MOVE_FORWARD...")
                smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
                tello.send_rc_control(0, 0, 0, 0)
                print(f"[{NAME}] MOVE_FORWARD DONE -> ACK")
                send_ack(msg_id)

            elif cmd == "LAP":
                log(f"{NAME} LAP semicircle R={RADIUS_CM}")
                print(f"[{NAME}] LAP (R={RADIUS_CM}cm)...")
                smooth_semicircle(tello, RADIUS_CM, V_CM_S, cw=True, hz=HZ)
                tello.send_rc_control(0, 0, 0, 0)
                print(f"[{NAME}] LAP DONE -> ACK")
                send_ack(msg_id)

            elif cmd == "LAND":
                log(f"{NAME} LAND")
                print(f"[{NAME}] LAND...")
                try:
                    tello.land()
                except Exception as e:
                    log(f"{NAME} land error: {e}")
                    print(f"[{NAME}] land error: {e}")
                print(f"[{NAME}] LAND DONE -> ACK")
                send_ack(msg_id)

            elif cmd in ("EMERGENCY", "KILL"):
                log(f"{NAME} !!! {cmd} !!!")
                print(f"[{NAME}] !!! {cmd} !!!")
                try:
                    tello.emergency()
                except Exception as e:
                    log(f"{NAME} emergency error: {e}")
                    print(f"[{NAME}] emergency error: {e}")
                send_ack(msg_id)

            else:
                log(f"{NAME} Unknown command {cmd}")
                print(f"[{NAME}] Unknown command: {cmd}")
                send_ack(msg_id)

        except Exception as e:
            log(f"{NAME} Error executing {cmd}: {e}")
            print(f"[{NAME}] Error executing {cmd}: {e}")
            # Still ACK so master doesn't hang forever
            send_ack(msg_id)

        finally:
            with seen_lock:
                completed_ids.add(msg_id)
            busy.release()

    while True:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue

        # Accept only from master ethernet IP (safety)
        if addr[0] != MASTER_IP:
            continue

        txt = data.decode(errors="ignore").strip()
        parts = txt.split("|")
        if len(parts) != 2:
            continue

        cmd, msg_id = parts
        log(f"{NAME} Received {cmd} id={msg_id} from {addr[0]}")
        print(f"[{NAME}] Received {cmd} from master")

        threading.Thread(target=handle, args=(cmd, msg_id), daemon=True).start()

if __name__ == "__main__":
    main()
