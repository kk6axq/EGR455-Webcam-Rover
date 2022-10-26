"""
tcp_test.py
October 25, 2022
Lukas Severinghaus, Carlos Chacon, Miguel Chacon, Salsabil Soliman

Tests TCP connectivity to the robot, for commanding movements.

Used for testing the EGR455 team project.
"""
import socket, struct
def init_tcp():
    global tcp_ang

    host = '192.168.0.134'
    port = 25002                   # The same port as used by the server
    tcp_ang = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_ang.connect((host, port))



def close_tcp():
    global tcp_ang
    tcp_ang.close()

def send_theta(value):
    global tcp_ang
    data = struct.pack("<d", value)
    tcp_ang.sendall(data)
    print("Sent bytes")
    
init_tcp()

send_theta(2)

close_tcp()
