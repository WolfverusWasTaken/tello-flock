#!/usr/bin/env python3
import time, socket, struct, math, threading
from djitellopy import Tello
import serial

# =====================
# CONFIG
# =====================
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

# Set per slave
MY_TAG_ID = 2           # change to 1 on slave1, 2 on slave2
OFFSET_XY = (-0.30, -0.20)  # slave1 example. slave2 set to (+0.30, -0.20)

ANCHOR_UART = "/dev/ttyUSB0"   # if slave also reads UWB from anchor (optional)
ANCHOR_BAUD = 921600
ANCHOR_FRAME_LEN = 67

TAKEOFF_Z = 1.6
CTRL_HZ = 20

VXY_MAX = 45
VZ_MAX  = 35

STOP = {"stop": False}
POSES = {}
LOCK = threading.Lock()

# =====================
# Optional UWB reader on slaves (recommended for closed-loop)
# If you don't have anchor UART on slave, comment this section and replace pose with dead-reckoning (not recommended).
# =====================
def int24_le(b: bytes) -> int:
    v = b[0] | (b[1] << 8) | (b[2] << 16)
    if v & 0x800000:
        v -= 1 << 24
    return v

def parse_anchor_frame0_best_effort(frame: bytes):
    now = time.time()
    i = 2
    while i + 11 <= len(frame):
        tid = frame[i]
        role = frame[i+1]
        if role == 0x02:
            x = int24_le(frame[i+2:i+5]) / 1000.0
            y = int24_le(frame[i+5:i+8]) / 1000.0
            z = int24_le(frame[i+8:i+11]) / 1000.0
            if all(-50 < v < 50 for v in (x,y,z)):
                with LOCK:
                    POSES[tid] = (x,y,z,now)
                i += 27
                continue
        i += 1

def uwb_reader():
    ser = serial.Serial(ANCHOR_UART, ANCHOR_BAUD, timeout=0.2)
    buf = bytearray()
    while not STOP["stop"]:
        buf += ser.read(256)
        while True:
            idx = buf.find(b"\x55")
            if idx < 0:
                if len(buf) > 4096: buf = buf[-256:]
                break
            if idx > 0: del buf[:idx]
            if len(buf) < ANCHOR_FRAME_LEN: break
            frame = bytes(buf[:ANCHOR_FRAME_LEN])
            del buf[:ANCHOR_FRAME_LEN]
            try:
                parse_anchor_frame0_best_effort(frame)
            except:
                pass
    ser.close()

def get_pose(tid):
    with LOCK:
        return POSES.get(tid)

# =====================
# Control helpers
# =====================
def clamp(x, lo, hi): return max(lo, min(hi, x))
def mps_to_rc(v_mps, vmax_cmps):
    return int(clamp(round(v_mps*100.0), -vmax_cmps, vmax_cmps))

def main():
    # Start local UWB reader if you have anchor UART on this slave
    threading.Thread(target=uwb_reader, daemon=True).start()

    # UDP receiver
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(0.2)

    # Tello
    tello = Tello()
    tello.connect()
    print("[SLAVE] battery:", tello.get_battery())
    tello.takeoff()

    dt = 1.0 / CTRL_HZ
    last_pkt = None

    try:
        while True:
            # receive latest setpoint
            try:
                data, _ = sock.recvfrom(1024)
                if len(data) >= struct.calcsize("<4sfffIff"):
                    magic, sx, sy, sz, seq, vx, vy = struct.unpack("<4sfffIff", data[:struct.calcsize("<4sfffIff")])
                    if magic == b"PSET":
                        last_pkt = (sx, sy, sz, vx, vy, seq, time.time())
            except socket.timeout:
                pass

            pose = get_pose(MY_TAG_ID)
            if not pose or not last_pkt:
                time.sleep(dt)
                continue

            x,y,z,_ = pose
            sx, sy, sz, vx_hint, vy_hint, seq, t_pkt = last_pkt

            # apply formation offset to setpoint
            dx = (sx + OFFSET_XY[0]) - x
            dy = (sy + OFFSET_XY[1]) - y

            # simple proportional velocity toward setpoint + small velocity hint
            vx = clamp(0.9*dx + 0.2*vx_hint, -0.8, 0.8)  # m/s
            vy = clamp(0.9*dy + 0.2*vy_hint, -0.8, 0.8)

            ez = (sz) - z
            vz = clamp(0.8*ez, -0.35, 0.35)

            lr = mps_to_rc(vx, VXY_MAX)  # x -> left/right
            fb = mps_to_rc(vy, VXY_MAX)  # y -> forward/back
            ud = mps_to_rc(vz, VZ_MAX)
            yw = 0

            tello.send_rc_control(lr, fb, ud, yw)
            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        STOP["stop"] = True
        tello.send_rc_control(0,0,0,0)
        tello.land()
        sock.close()
        print("[SLAVE] Landed.")

if __name__ == "__main__":
    main()