import socket
import datetime
from datetime import datetime, timedelta

def send(msg):
    sockSend.sendto(msg.encode('UTF-8'), (UDP_IP, UDP_REMOTE_PORT))

UDP_IP = "127.0.0.1"

UDP_REMOTE_PORT = 9808
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_REMOTE_PORT)
sockSend = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

UDP_LOCAL_PORT = 9810
print("UDP local port: %s" % UDP_LOCAL_PORT)
sockRec = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sockRec.bind((UDP_IP, UDP_LOCAL_PORT))


#HandShake
connection = False
wait_until = datetime.now() + timedelta(seconds=5)
send("SYN")
while not connection:
    data, addr = sockRec.recvfrom(1024) # buffer size is 1024 bytes
    if data.decode('UTF-8') == "ACK":
        connection = True
        send("SYNACK")
    elif wait_until < datetime.now():# or MAX_ATTEMPTS?
        wait_until = datetime.now() + timedelta(seconds=5)
        send("SYN")


print("Connected")

