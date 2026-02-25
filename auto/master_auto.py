#!/usr/bin/env python3
import math, time, socket, struct, threading
import serial
from djitellopy import Tello

# ---------------- CONFIG ----------------
MASTER_ORIN_IP = "192.168.50.1"
SLAVE1_ADDR = ("192.168.50.2", 6000)
SLAVE2_ADDR = ("192.168.50.3", 6000)

# TAG IDs
MASTER_TAG = 0
SLAVE1_TAG = 1
SLAVE2_TAG = 2

# Formation offsets (meters) in world frame (X=left/right, Y=forward/back)
SLAVE1_OFF = (-0.30, -0.20, 0.0)
SLAVE2_OFF = (+0.30, -0.20, 0.0)

# Path: C = A + offset (no “yaw on spot” ending)
C_OFFSET = (0.0, 1.0)   # meters (x,y)

# UWB Anchor (connected only to master)
ANCHOR_UART = "/dev/ttyUSB0"
ANCHOR_BAUD = 921600
ANCHOR_FRAME_LEN = 67   # adjust if your Anchor_Frame0 differs

# Flight tuning
TAKEOFF_Z = 1.6
ARRIVE_TOL = 0.20
CTRL_HZ = 20            # RC updates
VXY_MAX = 45            # cm/s
VZ_MAX  = 35            # cm/s
YAW = 0                 # keep yaw stable

# Smooth arc tuning
LOOKAHEAD = 0.60        # meters
TURN_RADIUS = 0.90      # meters
ARC_POINTS = 25

# ---------------- Shared pose cache ----------------
POSES = {}   # tid -> (x,y,z,t)
LOCK = threading.Lock()
STOP = {"stop": False}

def clamp(x, lo, hi): return max(lo, min(hi, x))
def norm(x,y): return math.hypot(x,y)
def unit(x,y):
    n = norm(x,y)
    return (0.0,0.0) if n < 1e-9 else (x/n, y/n)

def mps_to_rc(v_mps, vmax_cmps):
    return int(clamp(round(v_mps * 100.0), -vmax_cmps, vmax_cmps))

# ----------- UWB parsing (best-effort) -----------
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
        if role == 0x02:  # TAG
            x = int24_le(frame[i+2:i+5]) / 1000.0
            y = int24_le(frame[i+5:i+8]) / 1000.0
            z = int24_le(frame[i+8:i+11]) / 1000.0
            # Drop obvious invalids (your UI shows -8388 etc)
            if all(-50 < v < 50 for v in (x,y,z)) and not (abs(x)>8 and abs(y)>8 and abs(z)>8):
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

# ----------- Smooth “fillet” arc at B -----------
def generate_fillet_path(A, B, C, R, npts):
    v1 = unit(B[0]-A[0], B[1]-A[1])
    v2 = unit(C[0]-B[0], C[1]-B[1])
    dot = max(-1.0, min(1.0, v1[0]*v2[0] + v1[1]*v2[1]))
    theta = math.acos(dot)
    if theta < 1e-3:
        return [A,B,C]

    t = R * math.tan(theta/2.0)
    t = min(t, 0.45*norm(B[0]-A[0], B[1]-A[1]), 0.45*norm(C[0]-B[0], C[1]-B[1]))
    T1 = (B[0]-v1[0]*t, B[1]-v1[1]*t)
    T2 = (B[0]+v2[0]*t, B[1]+v2[1]*t)

    cross = v1[0]*v2[1] - v1[1]*v2[0]
    left = cross > 0
    n1 = (-v1[1], v1[0]) if left else (v1[1], -v1[0])
    n2 = (-v2[1], v2[0]) if left else (v2[1], -v2[0])

    # intersect T1+n1*s and T2+n2*u
    a11, a12 = n1[0], -n2[0]
    a21, a22 = n1[1], -n2[1]
    b1, b2 = (T2[0]-T1[0]), (T2[1]-T1[1])
    det = a11*a22 - a12*a21
    if abs(det) < 1e-6:
        return [A, T1, T2, C]

    s = (b1*a22 - b2*a12)/det
    cx, cy = (T1[0]+n1[0]*s, T1[1]+n1[1]*s)

    ang1 = math.atan2(T1[1]-cy, T1[0]-cx)
    ang2 = math.atan2(T2[1]-cy, T2[0]-cx)

    def wrap(a):
        while a < -math.pi: a += 2*math.pi
        while a >  math.pi: a -= 2*math.pi
        return a

    d = wrap(ang2-ang1)
    if left and d < 0: d += 2*math.pi
    if (not left) and d > 0: d -= 2*math.pi

    arc = []
    for k in range(npts):
        u = k/(npts-1) if npts>1 else 1.0
        ang = ang1 + d*u
        arc.append((cx + R*math.cos(ang), cy + R*math.sin(ang)))

    return [A, T1] + arc + [T2, C]

def pick_lookahead(cur, path, lookahead):
    best_i, best_d = 0, 1e9
    for i,p in enumerate(path):
        d = norm(p[0]-cur[0], p[1]-cur[1])
        if d < best_d:
            best_d, best_i = d, i
    dist = 0.0
    prev = cur
    for j in range(best_i, len(path)):
        p = path[j]
        dist += norm(p[0]-prev[0], p[1]-prev[1])
        prev = p
        if dist >= lookahead:
            return p
    return path[-1]

# ----------- RC controller (simple, stable) -----------
def rc_to_target(cur, tgt, z_tgt):
    x,y,z,_ = cur
    tx,ty = tgt

    dx, dy = (tx-x, ty-y)
    ux, uy = unit(dx, dy)
    dist = norm(dx, dy)

    # speed schedule (m/s)
    speed = clamp(dist * 0.8, 0.20, 0.80)
    vx, vy = (ux*speed, uy*speed)

    # altitude P-control (m/s)
    ez = z_tgt - z
    vz = clamp(0.8*ez, -0.35, 0.35)

    lr = mps_to_rc(vx, VXY_MAX)   # X -> left/right
    fb = mps_to_rc(vy, VXY_MAX)   # Y -> forward/back
    ud = mps_to_rc(vz, VZ_MAX)
    yw = YAW
    return lr, fb, ud, yw

# ----------- UDP send to slaves -----------
# Packet: b'RCMD' + int8 lr fb ud yaw + uint32 seq
PKT_FMT = "<4sbbbbI"

def send_rc(sock, addr, lr, fb, ud, yw, seq):
    pkt = struct.pack(PKT_FMT, b"RCMD", lr, fb, ud, yw, seq)
    sock.sendto(pkt, addr)

def wait_pose(tid, timeout=8.0):
    t0 = time.time()
    while time.time()-t0 < timeout:
        p = get_pose(tid)
        if p:
            return p
        time.sleep(0.05)
    raise TimeoutError(f"No pose for TAG {tid}")

def main():
    threading.Thread(target=uwb_reader, daemon=True).start()

    print("[MASTER] waiting for TAG poses...")
    pM = wait_pose(MASTER_TAG)
    p1 = wait_pose(SLAVE1_TAG)
    p2 = wait_pose(SLAVE2_TAG)
    print("[MASTER] got poses")

    # Define A, B, C in world frame
    A = (pM[0], pM[1])
    # EDIT B to your real target
    B = (A[0] + 2.0, A[1] + 2.0)
    C = (A[0] + C_OFFSET[0], A[1] + C_OFFSET[1])

    path = generate_fillet_path(A, B, C, TURN_RADIUS, ARC_POINTS)
    print(f"[MASTER] A={A} B={B} C={C} pts={len(path)}")

    # Connect master tello (local)
    tello = Tello()
    tello.connect()
    print("[MASTER] battery:", tello.get_battery())
    tello.takeoff()

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    seq = 0
    dt = 1.0 / CTRL_HZ
    final = path[-1]

    print("[MASTER] running control loop (Ctrl+C to land all)")
    try:
        while True:
            curM = get_pose(MASTER_TAG)
            curS1 = get_pose(SLAVE1_TAG)
            curS2 = get_pose(SLAVE2_TAG)
            if not (curM and curS1 and curS2):
                time.sleep(dt)
                continue

            # lookahead target for master along smooth path
            tgtM_xy = pick_lookahead((curM[0], curM[1]), path, LOOKAHEAD)

            # slave targets = master target + offsets
            tgtS1_xy = (tgtM_xy[0] + SLAVE1_OFF[0], tgtM_xy[1] + SLAVE1_OFF[1])
            tgtS2_xy = (tgtM_xy[0] + SLAVE2_OFF[0], tgtM_xy[1] + SLAVE2_OFF[1])

            # compute RCs
            lrM, fbM, udM, ywM = rc_to_target(curM, tgtM_xy, TAKEOFF_Z)
            lr1, fb1, ud1, yw1 = rc_to_target(curS1, tgtS1_xy, TAKEOFF_Z)
            lr2, fb2, ud2, yw2 = rc_to_target(curS2, tgtS2_xy, TAKEOFF_Z)

            # send to master drone directly
            tello.send_rc_control(lrM, fbM, udM, ywM)

            # send to slaves over LAN
            seq += 1
            send_rc(sock, SLAVE1_ADDR, lr1, fb1, ud1, yw1, seq)
            send_rc(sock, SLAVE2_ADDR, lr2, fb2, ud2, yw2, seq)

            # optional: stop at final
            if norm(final[0]-curM[0], final[1]-curM[1]) < ARRIVE_TOL:
                # keep holding position (or you can land)
                pass

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[MASTER] landing all...")
    finally:
        # stop all RC
        try:
            tello.send_rc_control(0,0,0,0)
            tello.land()
        except:
            pass

        # send "stop" to slaves
        for addr in (SLAVE1_ADDR, SLAVE2_ADDR):
            try:
                seq += 1
                send_rc(sock, addr, 0, 0, 0, 0, seq)
            except:
                pass
        sock.close()
        STOP["stop"] = True

if __name__ == "__main__":
    main()