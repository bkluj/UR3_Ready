import socket
import time
import struct
import numpy as np


HOST = "192.168.1.90"
PORT = 30002
PORTSTOP = 30003

g_time_sleep = 10

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

zero = "movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], a=0.2, v=0.1)"

def send_command(cmd):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send((cmd+"\n").encode("utf-8"))
    print(f"Data sent: {cmd[7:35]}, with {cmd[:5]}")
    s.close()

def wait_for_stop():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORTSTOP))
    time.sleep(g_time_sleep)

    while True:
        data = s.recv(1108)
        if len(data) < 1108:
            continue

        # ← offset może wymagać korekty, zależnie od wersji firmware
        start_byte = 252  # standardowo velocity zaczyna się od 252 (dla starszego UR3); dla UR3e często 756

        try:
            velocity = [
                struct.unpack(">d", data[start_byte + i * 8 : start_byte + (i + 1) * 8])[0]
                for i in range(6)
            ]
        except Exception as e:
            print("Błąd odczytu velocity:", e)
            continue

        print("velocity: ", ", ".join(f"{v:.2f}" for v in velocity))

        if max(abs(v) for v in velocity) < 0.01:
            break

        time.sleep(g_time_sleep)
    s.close()

# pozycja przed chwytem
# send_command("movej([-0.25, 0.35, 0.0, 0.5, 0.0, -0.80], a=0.2, v=0.1)")

# 1 pozycja
# send_command("movej([0.0, 0.35, 0.0, 0.0, 0.0, -0.35], a=0.2, v=0.1)")

# pozycja pośrednia
# send_command("movej([-0.5, 0.2, 0.0, 0.0, 0.0, -0.2], a=0.2, v=0.1)")

# pozycja końcowa
# send_command("movej([-0.5, 0.35, 0.0, 0.0, 0.0, -0.35], a=0.2, v=0.1)")


def move(typeofmove):
    if typeofmove == 1:
        # send_command("movej([-0.2, 0.2, 0.5, 0.5, -0.2, -1.3], a=0.2, v=0.3)")
        wait_for_stop()
    elif typeofmove == 2:
        send_command(f"movej([{np.pi/7}, -{np.pi/3 - np.pi/(3.04)}, {np.pi/3 - np.pi/4}, -{np.pi/2+np.pi/17}, -{np.pi/2- np.pi/4}, -1.4], a=0.2, v=0.3)")
        wait_for_stop()
    elif typeofmove == 3:
        send_command(f"movej([{0}, -{np.pi/3}, {np.pi/3}, -{np.pi/2}, -{np.pi/2}, -1.4], a=0.2, v=0.3)")
        wait_for_stop()
    elif typeofmove == 4:
        send_command(f"movej([{-np.pi/8-np.pi/35}, -{np.pi/3 - np.pi/(2.86)}, {np.pi/3 - np.pi/4-np.pi/100}, -{np.pi/2+np.pi/7.6}, -{np.pi/2- np.pi/4}, -1.4], a=0.2, v=0.3)")
        wait_for_stop()
    elif typeofmove == 0:
        send_command(f"movej([-0.0, {-np.pi/2}, 0.0, 0.0, {np.pi/2}, {np.pi/2}], a=0.2, v=0.3)")
        wait_for_stop()

move(0)

'''
def move(typeofmove):
    if typeofmove == 1:
        send_command("movej([-0.2, 0.2, 0.5, 0.5, -0.2, -1.3], a=0.2, v=0.3)")
        wait_for_stop()
    elif typeofmove == 2:
        send_command("movej([0.0, 0.2, 0.0, 1.3, 0.45, -1.4], a=0.2, v=0.3)")
        wait_for_stop()
    elif typeofmove == 3:
        send_command("movej([-0.35, 0.0, 0.0, 1.3, 0.45, -1.4], a=0.2, v=0.3)")
        wait_for_stop()
    else:
        send_command("movej([-0.7, 0.2, 0.0, 1.3, 0.45, -1.4], a=0.2, v=0.3)")
        wait_for_stop()
'''

