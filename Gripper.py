import socket
import time

# Konfiguracja połączenia
ROBOT_IP = "192.168.1.90"
PORT = 30002


# URScript do aktywacji, zamknięcia i otwarcia chwytaka
script = """
def control_gripper():
  rq_activate_and_wait()       
  sleep(1.0)
  rq_close()                  
  sleep(2.0)
  rq_open()                    
  sleep(1.0)
end

control_gripper()
"""

def send_urscript_to_ur3(script, ip, port):
    try:
        print(f"Łączenie z UR3 pod adresem {ip}:{port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ip, port))
        print("Połączono. Wysyłanie skryptu URScript...")
        sock.sendall(script.encode('utf8'))
        time.sleep(5)  # Poczekaj, aż skrypt się wykona
        sock.close()
        print("Zamknięto połączenie.")
    except Exception as e:
        print(f"Błąd połączenia: {e}")

if __name__ == "__main__":
    send_urscript_to_ur3(script, ROBOT_IP, PORT)
