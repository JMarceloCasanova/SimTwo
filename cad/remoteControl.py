import socket
import datetime
from datetime import datetime, timedelta
import time
import numpy as np
from tqdm import tqdm
import os.path
import random
import christofides

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

class aabb():
    def __init__(self, part=None, triangles=None, rot_mat=np.eye(3)):
        self.min_x = 999999
        self.max_x = -999999
        self.min_y = 999999
        self.max_y = -999999
        self.min_z = 999999
        self.max_z = -999999
        if (part is not None) and (triangles is not None):
            for j in part:
                center = rot_mat @ triangles[j].center
                if center[0] < self.min_x:
                    self.min_x = center[0]
                if center[0] > self.max_x:
                    self.max_x = center[0]
                if center[1] < self.min_y:
                    self.min_y = center[1]
                if center[1] > self.max_y:
                    self.max_y = center[1]
                if center[2] < self.min_z:
                    self.min_z = center[2]
                if center[2] > self.max_z:
                    self.max_z = center[2]
        elif (triangles is not None):
            for tri in triangles:
                center = rot_mat @ tri.center
                if center[0] < self.min_x:
                    self.min_x = center[0]
                if center[0] > self.max_x:
                    self.max_x = center[0]
                if center[1] < self.min_y:
                    self.min_y = center[1]
                if center[1] > self.max_y:
                    self.max_y = center[1]
                if center[2] < self.min_z:
                    self.min_z = center[2]
                if center[2] > self.max_z:
                    self.max_z = center[2]

    def reset(self):
        self.min_x = 999999
        self.max_x = -999999
        self.min_y = 999999
        self.max_y = -999999
        self.min_z = 999999
        self.max_z = -999999

    def volume(self):
        return (self.max_x-self.min_x) * (self.max_y-self.min_y) * (self.max_z-self.min_z)

    def center(self):
        return np.array([(self.max_x+self.min_x)/2,(self.max_y+self.min_y)/2,(self.max_z+self.min_z)/2], dtype=np.float64)

    def offset(self, val):
        self.min_x = self.min_x - val 
        self.min_y = self.min_y - val 
        self.min_z = self.min_z - val 
        self.max_x = self.max_x + val 
        self.max_y = self.max_y + val 
        self.max_z = self.max_z + val

    def write_file(self, filename="extents1.txt"):
        with open('cad/' + filename, 'w') as f:
            f.write(str(self.min_x)+" ")
            f.write(str(self.min_y)+" ")
            f.write(str(self.min_z)+"\n")
            f.write(str(self.max_x)+" ")
            f.write(str(self.max_y)+" ")
            f.write(str(self.max_z)+"\n")
        
def load_triangles(filename = "triangles.npy"):
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
    np.save(filename, triangles, allow_pickle=True)
    print("loaded and saved " + str(len(triangles)) +" triangles")

def disconnect():
    send("Disconnect")
    print("Disconnected")

def vec_length(a):
    return np.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

def angle(a, b):
    #print((a[0]*b[0] + a[1]*b[1] + a[2] + b[2]) / (vec_length(a)*vec_length(b)) )
    return np.arccos((a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (vec_length(a)*vec_length(b)) )

def cartesian_to_spherical(a):
    r = vec_length(a)
    theta = np.arctan2(np.sqrt(a[0]**2+a[1]**2),a[2])
    phi = np.arctan2(a[1],a[0])
    return np.array([r, theta, phi])

def spherical_to_cartesian(a):
    x = a[0]*np.cos(a[2])*np.sin(a[1])
    y = a[0]*np.sin(a[2])*np.sin(a[1])
    z = a[0]*np.cos(a[1])
    return np.array([x, y, z])

def subdivision(max_angle, triangles):# patch_creation
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
                if (angle(triangles[seed].normal, triangles[nei].normal) < max_angle) \
                    and (nei in free_triangles) and (nei not in to_be_visited):
                    to_be_visited.append(nei)
            #print("\t", len(to_be_visited))
        parts.append(part)

    return parts

def CalculateBoxRaster(BoxOffset, BoxUStep, BoxVExtend, ext, BoxSelectFace= 'ftop'):
    faceExt = aabb()
    if BoxSelectFace=='ftop':
        faceExt = ext
        faceExt.min_z = ext.max_z + BoxOffset
        faceExt.max_z = ext.max_z + BoxOffset
        faceExt.min_x = faceExt.min_x - BoxUStep
        faceExt.max_x = faceExt.max_x + BoxUStep
        faceExt.min_y = faceExt.min_y - BoxVExtend
        faceExt.max_y = faceExt.max_y + BoxVExtend
    traj = np.zeros( ( 2*round((faceExt.max_x - faceExt.min_x) / BoxUStep), 3) , dtype=np.float32)
    for i in range(np.shape(traj)[0] // 2):
        if i%2 == 0:
            traj[i*2][0] = faceExt.min_x + i*BoxUStep
            traj[i*2][1] = faceExt.min_y
            traj[i*2][2] = faceExt.max_z
            traj[i*2+1][0] = faceExt.min_x + i*BoxUStep
            traj[i*2+1][1] = faceExt.max_y
            traj[i*2+1][2] = faceExt.max_z
        else:
            traj[i*2][0] = faceExt.min_x + (i)*BoxUStep
            traj[i*2][1] = faceExt.max_y
            traj[i*2][2] = faceExt.max_z
            traj[i*2+1][0] = faceExt.min_x + (i)*BoxUStep
            traj[i*2+1][1] = faceExt.min_y
            traj[i*2+1][2] = faceExt.max_z
    return(traj)

def CalculatePatchyTrajectory(parts, triangles, save_extents=True):
    complete_traj = np.zeros((0, 6))
    total_extents = aabb(triangles=triangles)
    total_extents.offset(0.3)
    connection_radius = vec_length(np.array([total_extents.max_x, total_extents.max_y, total_extents.max_z]) - total_extents.center())
    print("connection_radius")
    print(connection_radius)

    for part in parts:
        if np.shape(complete_traj)[0] == 0:
            rob_piece_home = np.array([[total_extents.min_x, total_extents.min_y, total_extents.max_z, 0, 0, -1]])
            complete_traj = np.append(complete_traj, rob_piece_home, axis=0)
        
        align_rot_axis = np.cross(triangles[part[0]].normal, np.array([0,0,1]))
        if np.sqrt(np.sum(align_rot_axis**2)) < 0.0001:
            align_rot_axis = np.array([0,0,1])
            phi = 0
        else:
            align_rot_axis = align_rot_axis/np.sqrt(np.sum(align_rot_axis**2))
            phi = angle(triangles[part[0]].normal, np.array([0,0,1]))
        W = np.array([[0,                  -align_rot_axis[2], align_rot_axis[1]],\
                      [align_rot_axis[2],  0,                  -align_rot_axis[0]],\
                      [-align_rot_axis[1], align_rot_axis[0],  0]])
        align_rot_mat_z = np.eye(3) + np.sin(phi)*W + (1-np.cos(phi))*W@W
        
        align_rot_mat = np.eye(3)
        min_volume = 99999999
        aligned_phi = 0
        min_extents = aabb()
        for phi in np.arange(0, 2*np.pi, np.pi/180):
            align_rot_mat_temp = np.array([[np.cos(phi), -np.sin(phi), 0],\
                                            [np.sin(phi), np.cos(phi), 0],\
                                            [0, 0, 1]])
            extents = aabb()
            for j in part:
                #new_center = np.matmul(align_rot_mat_temp,np.matmul(align_rot_mat_z,triangles[j].center))
                new_center = align_rot_mat_temp @ align_rot_mat_z @ triangles[j].center
                if new_center[0] < extents.min_x:
                    extents.min_x = new_center[0]
                if new_center[0] > extents.max_x:
                    extents.max_x = new_center[0]
                if new_center[1] < extents.min_y:
                    extents.min_y = new_center[1]
                if new_center[1] > extents.max_y:
                    extents.max_y = new_center[1]
                if new_center[2] < extents.min_z:
                    extents.min_z = new_center[2]
                if new_center[2] > extents.max_z:
                    extents.max_z = new_center[2]
            if extents.max_z-extents.min_z < 0.001: #avoid volume==0
                extents.min_z = extents.max_z - 0.1
            volume = extents.volume()
            if volume < min_volume:
                min_volume = volume
                aligned_phi = phi
                min_extents = extents
                #align_rot_mat = np.matmul(align_rot_mat_temp, align_rot_mat_z) #Don't mess this up!
                align_rot_mat = align_rot_mat_temp @ align_rot_mat_z #Don't mess this up!
        if save_extents:
            min_extents.write_file("extents_"+str(np.where(parts==part)))

        #Got align_rot_mat and min_extents
        BoxSelectFace = 'fTop'
        BoxOffset = 0.5
        BoxUStep = 0.1
        BoxVExtend = 0.3
        traj = CalculateBoxRaster(BoxOffset, BoxUStep, BoxVExtend, min_extents)
        #reset_rot_mat = np.linalg.inv(align_rot_mat)
        reset_rot_mat = np.transpose(align_rot_mat)
        #print(traj)
        traj = np.array([(reset_rot_mat @ traj[i]) for i in range(np.shape(traj)[0])])# Am I this good?
        for point in traj:
            if point[2] < 0.01:
                point[2] = 0.01
        traj_normal = -triangles[part[0]].normal[np.newaxis]
        traj = np.append(traj, np.repeat(traj_normal, np.shape(traj)[0],axis=0), axis=1  )

        #connection between trajs
        start_point = complete_traj[np.shape(complete_traj)[0]-1]
        end_point = traj[0]
        start_point_spherical = cartesian_to_spherical(start_point[0:3])
        end_point_spherical = cartesian_to_spherical(end_point[0:3])
        spherical_dist_ang = end_point_spherical[1:3] - start_point_spherical[1:3]
        target_step_angle = np.array([np.pi/10, np.pi/10])
        n_steps = max(np.max(spherical_dist_ang/target_step_angle), np.min(spherical_dist_ang/target_step_angle), key=abs).astype(int)
        step_angle = spherical_dist_ang/n_steps
        if n_steps < 0:
            step_angle = -1*step_angle
            n_steps = -1*n_steps
        
        connection_traj = np.zeros((n_steps,3), dtype=np.float32)
        for i in range(0, n_steps):
            connection_traj[i] = spherical_to_cartesian( np.append(connection_radius, start_point_spherical[1:3]+(i*step_angle)))
        connection_traj = np.append(connection_traj, np.repeat(np.array([[0,0,-1]]), np.shape(connection_traj)[0],axis=0), axis=1  )
        if (traj_normal[0][2] != 1):
            complete_traj = np.append(complete_traj, connection_traj, axis=0)
            complete_traj = np.append(complete_traj, traj, axis=0)
            print()
            print()
            print("new Part:")
            print()
            print("triangles[part[0]].normal")
            print(triangles[part[0]].normal)
            #print("align_rot_axis")
            #print(align_rot_axis)
            #print("phi")
            #print(phi)
            #print("min_volume")
            #print(min_volume)
            #print("align rot mat:")
            #print(align_rot_mat)
            #print("test vector(1,1,1)")
            #print(align_rot_mat @ np.array([1,1,1]))
            #print("det(align_rot_mat) " + str(np.linalg.det(align_rot_mat)))
            #print("traj")
            #print(traj)
            #print("n_steps")
            #print(n_steps)
            #print("spherical_dist_ang")
            #print(spherical_dist_ang)
            #print("target_step_angle")
            #print(target_step_angle)
            #print("step_angle")
            #print(step_angle)
            #print("start_point")
            #print(start_point)
            #print("end_point")
            #print(end_point)
            #print("connection_traj")
            #print(connection_traj)
        else:
            print()
            print()
            print("down part")
            print()

    return complete_traj

def CalculatePatchyTrajectoryChristofides(parts, triangles, save_extents=True):
    complete_traj = np.zeros((0, 6))
    complete_traj_parts = []

    total_extents = aabb(triangles=triangles)
    total_extents.offset(0.3)
    connection_radius = vec_length(np.array([total_extents.max_x, total_extents.max_y, total_extents.max_z]) - total_extents.center())
    print("connection_radius")
    print(connection_radius)

    for part in parts:
        if np.shape(complete_traj)[0] == 0:
            rob_piece_home = np.array([[total_extents.min_x, total_extents.min_y, total_extents.max_z, 0, 0, -1]])
            complete_traj = np.append(complete_traj, rob_piece_home, axis=0)
        
        align_rot_axis = np.cross(triangles[part[0]].normal, np.array([0,0,1]))
        if np.sqrt(np.sum(align_rot_axis**2)) < 0.0001:
            align_rot_axis = np.array([0,0,1])
            phi = 0
        else:
            align_rot_axis = align_rot_axis/np.sqrt(np.sum(align_rot_axis**2))
            phi = angle(triangles[part[0]].normal, np.array([0,0,1]))
        W = np.array([[0,                  -align_rot_axis[2], align_rot_axis[1]],\
                      [align_rot_axis[2],  0,                  -align_rot_axis[0]],\
                      [-align_rot_axis[1], align_rot_axis[0],  0]])
        align_rot_mat_z = np.eye(3) + np.sin(phi)*W + (1-np.cos(phi))*W@W
        
        align_rot_mat = np.eye(3)
        min_volume = 99999999
        aligned_phi = 0
        min_extents = aabb()
        for phi in np.arange(0, 2*np.pi, np.pi/180):
            align_rot_mat_temp = np.array([[np.cos(phi), -np.sin(phi), 0],\
                                            [np.sin(phi), np.cos(phi), 0],\
                                            [0, 0, 1]])
            extents = aabb()
            for j in part:
                #new_center = np.matmul(align_rot_mat_temp,np.matmul(align_rot_mat_z,triangles[j].center))
                new_center = align_rot_mat_temp @ align_rot_mat_z @ triangles[j].center
                if new_center[0] < extents.min_x:
                    extents.min_x = new_center[0]
                if new_center[0] > extents.max_x:
                    extents.max_x = new_center[0]
                if new_center[1] < extents.min_y:
                    extents.min_y = new_center[1]
                if new_center[1] > extents.max_y:
                    extents.max_y = new_center[1]
                if new_center[2] < extents.min_z:
                    extents.min_z = new_center[2]
                if new_center[2] > extents.max_z:
                    extents.max_z = new_center[2]
            if extents.max_z-extents.min_z < 0.001: #avoid volume==0
                extents.min_z = extents.max_z - 0.1
            volume = extents.volume()
            if volume < min_volume:
                min_volume = volume
                aligned_phi = phi
                min_extents = extents
                #align_rot_mat = np.matmul(align_rot_mat_temp, align_rot_mat_z) #Don't mess this up!
                align_rot_mat = align_rot_mat_temp @ align_rot_mat_z #Don't mess this up!
        if save_extents:
            min_extents.write_file("extents_"+str(np.where(parts==part)))

        #Got align_rot_mat and min_extents
        BoxSelectFace = 'fTop'
        BoxOffset = 0.5
        BoxUStep = 0.1
        BoxVExtend = 0.3
        traj = CalculateBoxRaster(BoxOffset, BoxUStep, BoxVExtend, min_extents)
        #reset_rot_mat = np.linalg.inv(align_rot_mat)
        reset_rot_mat = np.transpose(align_rot_mat)
        #print(traj)
        traj = np.array([(reset_rot_mat @ traj[i]) for i in range(np.shape(traj)[0])])# Am I this good?
        for point in traj:
            if point[2] < 0.01:
                point[2] = 0.01
        traj_normal = -triangles[part[0]].normal[np.newaxis]
        traj = np.append(traj, np.repeat(traj_normal, np.shape(traj)[0],axis=0), axis=1 )

        
        
        if (traj_normal[0][2] != 1):
            #complete_traj = np.append(complete_traj, connection_traj, axis=0)
            #complete_traj = np.append(complete_traj, traj, axis=0)
            complete_traj_parts.append(traj)
            print()
            print()
            print("new Part:")
            print()
            print("triangles[part[0]].normal")
            print(triangles[part[0]].normal)
            #print("align_rot_axis")
            #print(align_rot_axis)
            #print("phi")
            #print(phi)
            #print("min_volume")
            #print(min_volume)
            #print("align rot mat:")
            #print(align_rot_mat)
            #print("test vector(1,1,1)")
            #print(align_rot_mat @ np.array([1,1,1]))
            #print("det(align_rot_mat) " + str(np.linalg.det(align_rot_mat)))
            #print("traj")
            #print(traj)
            #print("n_steps")
            #print(n_steps)
            #print("spherical_dist_ang")
            #print(spherical_dist_ang)
            #print("target_step_angle")
            #print(target_step_angle)
            #print("step_angle")
            #print(step_angle)
            #print("start_point")
            #print(start_point)
            #print("end_point")
            #print(end_point)
            #print("connection_traj")
            #print(connection_traj)
        else:
            print()
            print()
            print("down part")
            print()

    tsp_points = []
    for traj in complete_traj_parts:
        traj_center = cartesian_to_spherical((traj[0][0:3] + traj[-1][0:3])/2)[1:3]
        print("center")
        print(traj_center)
        tsp_points.append(traj_center)
    length, path = christofides.tsp(tsp_points)
    path = list(dict.fromkeys(path)) #to remove duplicates (in the end)

    for i in path:
        #connection between trajs
        start_point = complete_traj[np.shape(complete_traj)[0]-1]
        end_point = complete_traj_parts[i]
        start_point_spherical = cartesian_to_spherical(start_point[0:3])
        end_point_spherical = cartesian_to_spherical(end_point[0:3])
        spherical_dist_ang = end_point_spherical[1:3] - start_point_spherical[1:3]
        target_step_angle = np.array([np.pi/10, np.pi/10])
        n_steps = max(np.max(spherical_dist_ang/target_step_angle), np.min(spherical_dist_ang/target_step_angle), key=abs).astype(int)
        step_angle = spherical_dist_ang/n_steps
        if n_steps < 0:
            step_angle = -1*step_angle
            n_steps = -1*n_steps
        connection_traj = np.zeros((n_steps,3), dtype=np.float32)
        for i in range(0, n_steps):
            connection_traj[i] = spherical_to_cartesian( np.append(connection_radius, start_point_spherical[1:3]+(i*step_angle)))
        connection_traj = np.append(connection_traj, np.repeat(np.array([[0,0,-1]]), np.shape(connection_traj)[0],axis=0), axis=1  )
        

        complete_traj = np.append(complete_traj, connection_traj, axis=0)
        complete_traj = np.append(complecomplete_traj_parts[i], traj, axis=0)

    print("length")
    print(length)
    print("path")
    print(path)



    return complete_traj

def CalculateCrudePatchyTrajectory(parts, triangles, save_extents=True):
    complete_traj = np.zeros((0, 3))
    print("calculation trajectory for "+str(len(parts))+" parts")
    for part in parts:
        print()
        print()
        print("new Part:")
        print()
        print("triangles[part[0]].normal")
        print(triangles[part[0]].normal)
        align_rot_axis = np.cross(triangles[part[0]].normal, np.array([0,0,1]))
        if np.sqrt(np.sum(align_rot_axis**2)) < 0.0001:
            align_rot_axis = np.array([0,0,1])
            phi = 0
        else:
            align_rot_axis = align_rot_axis/np.sqrt(np.sum(align_rot_axis**2))
            phi = angle(triangles[part[0]].normal, np.array([0,0,1]))
        print("align_rot_axis")
        print(align_rot_axis)
        print("phi")
        print(phi)
        W = np.array([[0,                  -align_rot_axis[2], align_rot_axis[1]],\
                      [align_rot_axis[2],  0,                  -align_rot_axis[0]],\
                      [-align_rot_axis[1], align_rot_axis[0],  0]])
        align_rot_mat_z = np.eye(3) + np.sin(phi)*W + (1-np.cos(phi))*W@W
        
        align_rot_mat = align_rot_mat_z 
        
        #Got align_rot_mat and min_extents
        BoxSelectFace = 'fTop'
        BoxOffset = 0.5
        BoxUStep = 0.1
        BoxVExtend = 0.3
        print("align rot mat:")
        print(align_rot_mat)
        print("test vector(1,1,1)")
        print(align_rot_mat @ np.array([1,1,1]))
        print("det(align_rot_mat) " + str(np.linalg.det(align_rot_mat)))
        traj = CalculateBoxRaster(BoxOffset, BoxUStep, BoxVExtend, aabb(part, triangles, align_rot_mat))
        #reset_rot_mat = np.linalg.inv(align_rot_mat)
        reset_rot_mat = np.transpose(align_rot_mat)
        #print(traj)
        #traj = np.array([np.matmul(reset_rot_mat, traj[i]) for i in range(np.shape(traj)[0])])# Am I this good?
        traj = np.array([(reset_rot_mat @ traj[i]) for i in range(np.shape(traj)[0])])# Am I this good?
        #print(traj)
        complete_traj = np.append(complete_traj, traj, axis=0)
        
    return complete_traj

if __name__ == "__main__":
    paint_target = "cube45_chris"

    paint_target_tri = "cad/" + paint_target + "_tri.npy"
    if (not os.path.isfile(paint_target_tri)):
        connect()
        load_triangles(paint_target_tri)
        disconnect()

    triangles = np.load(paint_target_tri, allow_pickle=True)
    print("Loaded "+str(len(triangles))+" triangles")

    paint_target_parts = "cad/" + paint_target + "_parts.npy"
    if (not os.path.isfile(paint_target_parts)):
        parts = subdivision(np.pi/3, triangles)
        np.save(paint_target_parts, np.array([np.array(part) for part in parts], dtype=object), allow_pickle=True)
    parts = np.load(paint_target_parts, allow_pickle=True)
    print("Loaded "+ str(parts.shape[0]) + " parts")

    for part in parts:
        part_color = np.array([random.randint(0,254),random.randint(0,254),random.randint(0,254)], dtype=np.float64)
        for i in part:
            triangles[i].color = part_color

    #max_part = parts.index(sorted(parts, key=len)[-2])
    ##max_part = parts.index(max(parts, key=len))

    #part_color = np.array([random.randint(0,254),random.randint(0,254),random.randint(0,254)], dtype=np.float64)
    #max_color = np.array([random.randint(0,254),random.randint(0,254),random.randint(0,254)], dtype=np.float64)
    #for part in parts:
    #    if parts.index(part) == max_part:
    #        for i in part:
    #            triangles[i].color = max_color
    #    else:
    #        for i in part:
    #            triangles[i].color = part_color

    paint_target_result = "cad/"+paint_target+"_result.txt"
    with open(paint_target_result, 'w') as f:
        for i in range(len(triangles)):
            for j in range(3):
                f.write(str(triangles[i].color[j])+" ")
            f.write("\n")

    #traj = CalculatePatchyTrajectory(parts, triangles)
    #traj = CalculateCrudePatchyTrajectory(parts, triangles)
    traj = CalculatePatchyTrajectoryChristofides(parts, triangles)

    paint_target_traj = "cad/"+paint_target+"_traj.txt"
    with open(paint_target_traj, 'w') as f:
        for i in range(np.shape(traj)[0]):
            for j in range(6):
                f.write(str(traj[i][j])+" ")
            f.write("\n")

    print("done!")
    print("Files:")
    print(paint_target_tri)
    print(paint_target_parts)
    print(paint_target_result)
    print(paint_target_traj)