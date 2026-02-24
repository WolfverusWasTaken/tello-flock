#!/usr/bin/env python3
import socket, threading, time, math
from djitellopy import Tello

SELF_IP   = "192.168.50.3"
MASTER_IP = "192.168.50.1"
PORT = 5005
NAME = "SLAVE_RIGHT"

# ===== FLIGHT CONFIG =====
TAKEOFF_UP_CM = 120
V_CM_S = 20
HZ = 20
STRAIGHT_CM = 300

# tighter center radius 120cm, right offset +30cm => 150cm
RADIUS_CM = 150

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

def do_one_lap(tello: Tello):
    smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
    smooth_semicircle(tello, RADIUS_CM, V_CM_S, cw=True, hz=HZ)
    smooth_straight(tello, STRAIGHT_CM, V_CM_S, HZ)
    smooth_semicircle(tello, RADIUS_CM, V_CM_S, cw=True, hz=HZ)

def main():
    tello = Tello()
    tello.connect()
    print(f"[{NAME}] Connected. Battery={tello.get_battery()}%")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SELF_IP, PORT))
    print(f"[{NAME}] Listening {SELF_IP}:{PORT}")

    busy = threading.Lock()

    def send_ack(msg_id: str):
        sock.sendto(f"ACK|{msg_id}|{NAME}".encode(), (MASTER_IP, PORT))

    def handle(cmd: str, msg_id: str):
        if not busy.acquire(blocking=False):
            send_ack(msg_id)
            return
        try:
            if cmd == "TAKEOFF":
                print(f"[{NAME}] TAKEOFF")
                tello.takeoff()
                if TAKEOFF_UP_CM > 0:
                    tello.move_up(TAKEOFF_UP_CM)
                tello.send_rc_control(0, 0, 0, 0)
                time.sleep(0.3)
                send_ack(msg_id)  # TAKEOFF COMPLETE

            elif cmd == "RUN_LAP":
                print(f"[{NAME}] RUN_LAP")
                send_ack(msg_id)
                do_one_lap(tello)

            elif cmd == "LAND":
                print(f"[{NAME}] LAND")
                try:
                    tello.land()
                except Exception as e:
                    print(f"[{NAME}] land error:", e)
                send_ack(msg_id)

            elif cmd == "EMERGENCY":
                print(f"[{NAME}] !!! EMERGENCY !!!")
                try:
                    tello.emergency()
                except Exception as e:
                    print(f"[{NAME}] emergency error:", e)
                send_ack(msg_id)

            else:
                send_ack(msg_id)
        finally:
            busy.release()

    while True:
        data, addr = sock.recvfrom(1024)
        if addr[0] != MASTER_IP:
            continue
        txt = data.decode(errors="ignore").strip()
        parts = txt.split("|")
        if len(parts) != 2:
            continue
        cmd, msg_id = parts
        threading.Thread(target=handle, args=(cmd, msg_id), daemon=True).start()

if __name__ == "__main__":
    main()