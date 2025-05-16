import socket
import time

HOST = "192.168.1.90"
PORT = 30002
PORTSTOP = 30003

g_time_sleep = 0.1

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
    time.sleep(g_time_sleep) # softcoded

    while True:
        data = s.recv(1108)
        if len(data) < 1108:
            continue
        velocity = [float.fromhex(data[252+i*8:255+(i+1)*8].hex()) for i in range(6)]
        if max(abs(v) for v in velocity) < 0.01:
            break
        time.sleep(g_time_sleep) # softcoded
    s.close()

send_command("movej([0.1, 0.1, 0.1, 0.0, 0.0, 0.0], a=0.2, v=0.1)")
wait_for_stop()
send_command(zero)
wait_for_stop()
send_command("movej([0.1, 0.1, 0.1, 0.0, 0.0, 0.0], a=0.2, v=0.1)")
wait_for_stop()
send_command(zero)
wait_for_stop()
send_command("movej([0.1, 0.1, 0.1, 0.0, 0.0, 0.0], a=0.2, v=0.1)")
wait_for_stop()
send_command(zero)
wait_for_stop()


