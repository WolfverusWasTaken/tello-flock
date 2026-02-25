#!/usr/bin/env python3
import socket, struct, time
from djitellopy import Tello

LISTEN_PORT = 6000
PKT_FMT = "<4sbbbbI"
PKT_LEN = struct.calcsize(PKT_FMT)

def main():
    # connect local tello
    tello = Tello()
    tello.connect()
    print("[SLAVE] battery:", tello.get_battery())
    tello.takeoff()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.settimeout(1.0)

    last_rx = time.time()

    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
                if len(data) < PKT_LEN:
                    continue
                magic, lr, fb, ud, yw, seq = struct.unpack(PKT_FMT, data[:PKT_LEN])
                if magic != b"RCMD":
                    continue

                tello.send_rc_control(int(lr), int(fb), int(ud), int(yw))
                last_rx = time.time()

            except socket.timeout:
                # if master stops sending, hold position (or land)
                if time.time() - last_rx > 2.0:
                    tello.send_rc_control(0,0,0,0)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            tello.send_rc_control(0,0,0,0)
            tello.land()
        except:
            pass
        sock.close()
        print("[SLAVE] Landed.")

if __name__ == "__main__":
    main()