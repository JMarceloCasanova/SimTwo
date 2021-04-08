import socket
import datetime
from datetime import datetime, timedelta
import time
import numpy as np
from tqdm import tqdm
import os.path
import random

UDP_IP = "127.0.0.1"
UDP_REMOTE_PORT = 9808

def send(msg):
    #print("send:", msg)
    sockSend.sendto(msg.encode('UTF-8'), (UDP_IP, UDP_REMOTE_PORT))
def read():
    data, _ = sockRec.recvfrom(64000)
    data = data.decode('UTF-8')
    #print("rec:", data)
    if data[0] == 'a':
        data = data[1:]
    elif data[0] == 'b':
        data = data.split("b")
        data = data[1:]
        for txt in data:
            txt = txt[1:]
            
    return data

def connect():
    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_REMOTE_PORT)
    global sockSend
    sockSend = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

    UDP_LOCAL_PORT = 9810
    print("UDP local port: %s" % UDP_LOCAL_PORT)
    global sockRec
    sockRec = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sockRec.bind((UDP_IP, UDP_LOCAL_PORT))


    #HandShake
    connection = False
    wait_until = datetime.now() + timedelta(seconds=1)
    send("SYN")
    while not connection:
        data, _ = sockRec.recvfrom(1024) # buffer size is 1024 bytes
        #print(data.decode('UTF-8'))
        if data.decode('UTF-8') == "ACK":
            connection = True
            time.sleep(1)
            send("SYNACK")
            time.sleep(1)
        elif wait_until < datetime.now():# or MAX_ATTEMPTS?
            wait_until = datetime.now() + timedelta(seconds=1)
            send("SYN")


    print("Connected")

class triangle(object):
    def __init__(self):
        self.vertices = [np.array([0,0,0], dtype=np.float64), np.array([0,0,0], dtype=np.float64), np.array([0,0,0], dtype=np.float64)] 
        self.normal = np.array([0,0,0], dtype=np.float64)
        self.center = np.array([0,0,0], dtype=np.float64)
        self.area = 0
        self.neighbors = []
        self.color = np.array([255,255,255], dtype=np.int16)

    def __str__(self):#custom print
        txt = "\nvertices:\n"
        for i in range(3):
            txt += str(self.vertices[i]) + "\n"
        txt += "normal\n"
        txt += str(self.normal)
        txt += "\ncenter\n"
        txt += str(self.center)
        txt += "\narea\n"
        txt += str(self.area)
        txt += "\nneighbors\n"
        txt += str(self.neighbors)
        return txt

def load_triangles():
    send("ReadTrianglesCount")
    count = int(read())
    #print(count)
    triangles = np.ndarray((count,),dtype=object)

    for i in range(count):
        triangles[i] = triangle()
    time.sleep(1)

    txt = "Read80Triangles "
    index = []
    for it in tqdm(range(0, count, 80)):
        send(txt + str(it))
        values = read()
        for i in range((count-it) if (it+80>count) else (80)):
            index.append(int(values.pop(0)))
            for j in range(3):
                for k in range(3):
                    triangles[i+it].vertices[j][k] = float(values.pop(0))
            for j in range(3):
                triangles[i+it].normal[j] = float(values.pop(0))
            for j in range(3):
                triangles[i+it].center[j] = float(values.pop(0))
            triangles[i+it].area = float(values.pop(0))
            num_neighbors = int(values.pop(0))
            for j in range(num_neighbors):
                triangles[i+it].neighbors.append(int(values.pop(0)))
            index.append(int(values.pop(0)))
    np.save("triangles", triangles, allow_pickle=True)
    print("loaded and saved " + str(len(triangles)) +" triangles")

def disconnect():
    send("Disconnect")
    print("Disconnected")

def vec_length(a):
    return np.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

def angle(a, b):
    #print((a[0]*b[0] + a[1]*b[1] + a[2] + b[2]) / (vec_length(a)*vec_length(b)) )
    return np.arccos((a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (vec_length(a)*vec_length(b)) )

def subdivision(max_angle, triangles):
    free_triangles = [i for i in range(len(triangles))]
    parts = []
    while len(free_triangles) > 0:
        part = []
        print()
        print(len(free_triangles))
        print(len(parts))
        print()
        part_seed = free_triangles[random.randint(0,len(free_triangles)-1)]
        to_be_visited = [part_seed]
        while len(to_be_visited) > 0:
            seed = to_be_visited.pop()
            free_triangles.remove(seed)
            part.append(seed)
            for nei in triangles[seed].neighbors:
                if (angle(triangles[part_seed].normal, triangles[nei].normal) < max_angle) \
                    and (nei in free_triangles) and (nei not in to_be_visited):
                    to_be_visited.append(nei)
            #print("\t", len(to_be_visited))
        parts.append(part)

    return parts

if __name__ == "__main__":
    if (not os.path.isfile('triangles.npy')) or False:
        connect()
        load_triangles()
        disconnect()

    triangles = np.load("triangles.npy", allow_pickle=True)
    print(len(triangles))
    
    parts = subdivision(np.pi/2, triangles)

    for part in parts:
        part_color = np.array([random.randint(0,254),random.randint(0,254),random.randint(0,254)], dtype=np.float64)
        for i in part:
            triangles[i].color = part_color

    #connect()
    #print("WriteCountResultColors "+str(len(triangles)))
    #send("WriteCountResultColors "+str(len(triangles))+" ")
    #time.sleep(3)
    #send("WriteResultColors ")
    #for i in tqdm(range(len(triangles))):
    #    if (i % 80)==0:
    #        time.sleep(2)
    #    for j in range(3):
    #        send("WriteResultColor "+str(triangles[i].color[j])+" ")
    #time.sleep(3)
    #disconnect()

    #with open('cad/ResultColors4.txt', 'w') as f:
    #    for i in range(len(triangles)):
    #        for j in range(3):
    #            f.write(str(triangles[i].color[j])+" ")
    #        f.write("\n")
    

    max_part = parts.index(max(parts, key=len))
    max_part = parts.index(max(parts, key=len))

    part_color = np.array([random.randint(0,254),random.randint(0,254),random.randint(0,254)], dtype=np.float64)
    max_color = np.array([random.randint(0,254),random.randint(0,254),random.randint(0,254)], dtype=np.float64)
    for part in parts:
        if parts.index(part) == max_part:
            for i in part:
                triangles[i].color = max_color
        else:
            for i in part:
                triangles[i].color = part_color

    with open('cad/ResultColors4.txt', 'w') as f:
        for i in range(len(triangles)):
            for j in range(3):
                f.write(str(triangles[i].color[j])+" ")
            f.write("\n")

    #for t in parts:
    i = parts[max_part][0]
    align_rot_axis = np.cross(triangles[i].normal, np.array([0,0,1]))
    align_rot_axis = align_rot_axis/np.sqrt(np.sum(align_rot_axis**2))

    phi = angle(triangles[i].normal, np.array([0,0,1]))
    W = np.array([[0, -align_rot_axis[2], align_rot_axis[1]],\
                    [align_rot_axis[2], 0, -align_rot_axis[0]],\
                    [-align_rot_axis[1], align_rot_axis[0], 0]])
    align_rot_mat_z = np.eye(3) + np.sin(phi)*W + (1-np.cos(phi))*W*W
    
    align_rot_mat = np.eye(3)
    min_volume = 99999999
    aligned_phi = 0
    for phi in np.arange(0, 2*np.pi, np.pi/180):
        align_rot_mat_temp = np.array([[np.cos(phi), -np.sin(phi), 0],\
                                        [np.sin(phi), np.cos(phi), 0],\
                                        [0, 0, 1]])
        aabb_min_x = 999999
        aabb_max_x = -999999
        aabb_min_y = 999999
        aabb_max_y = -999999
        aabb_min_z = 999999
        aabb_max_z = -999999
        for j in parts[max_part]:
            new_center = align_rot_mat_temp*align_rot_mat_z*triangles[parts[max_part]].center
            if new_center[0] < aabb_min_x :
                aabb_min_x = new_center[0]
            if new_center[0] > aabb_max_x :
                aabb_max_x = new_center[0]
            if new_center[1] < aabb_min_y :
                aabb_min_y = new_center[1]
            if new_center[1] > aabb_max_y :
                aabb_max_y = new_center[1]
            if new_center[2] < aabb_min_z :
                aabb_min_z = new_center[2]
            if new_center[2] > aabb_max_z :
                aabb_max_z = new_center[2]

        volume = (aabb_max_x-aabb_min_x) * (aabb_max_y-aabb_min_y) * (aabb_max_z-aabb_min_z)
        if volume < min_volume:
            min_volume = volume
            aligned_phi = phi
            align_rot_mat = align_rot_mat_temp*align_rot_mat_z #Don't mess this up!

