import cv2, numpy, time, os, threading, socket, imutils, serial, math
from unicorn2 import *
from transform import *
import bicorn
import numpy as np
import BGsub, BGsub_frank, recon_median, frankenstein, DBSCAN, search_chess, search_marker, colormap
backlog = 1
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('127.0.0.1', 12345))
s.listen(backlog)
print("Waiting for Unity")
client, address = s.accept()
print("Connected")
state_name = ['setting', 'captureTemplate', 'captureWorkspace', 'reconstruction', 'segmentation', 'locateMarker', 'generatePath', 'visualize']
### Parameters ###
USE_CUDA, AUTO_RUN, CIRCULAR_ROUND = True, True, 2
cap = Camera()
run, state, previous_state = 1, 1, 1
event = {'capture':0, 'circular':0, 'connect':0, 'connectXY':0, 'connectZ':0, 'home':0, 'homeXY':0, 'homeZ':0, 'circular_running':0, 'recon':0, 'segment':0, 'locateMarker':0, 'generatePath':0, 'visualize':0}
robot_event = {'homing':0, 'homingXY':0, 'homedXY':0, 'homingZ':0, 'homedZ':0, 'trajectoryXY_running':0, 'trajectoryXY_arrived':0, 'trajectoryZ_running':0, 'trajectoryZ_arrived':0, 'readedXY':0}
captured_image = {'warped':None, 'template_raw':None, 'template':None, 'reconstructed':None, 'segment':None, 'locateMarker':None, 'generatePath': None, 'visualize': None, 'chessMask': None, 'startMask':None}
chess_center, start_center = None, None
fourcc = cv2.VideoWriter_fourcc(*'XVID')
standby_image = imutils.resize(cv2.imread("bicorn.jpg"), height=800)
use_standby = True
recon_mode, segment_mode = 6, 0
command_queue = []
z_offset = 60
packets = []
packets_esp = []
packet_readXY = []
STEP_PER_ROUND = 800
def apply_checksum(l):
    checkSumOrdList = l[2:]
    checkSumOrdListSum = sum(checkSumOrdList)
    computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
    l.append(computedCheckSum)
class Robot():
    def __init__(self, comport_pic, baudrate_pic, comport_bt, baudrate_bt):
        self.comport_pic = comport_pic
        self.baudrate_pic = baudrate_pic
        self.comport_bt = comport_bt
        self.baudrate_bt = baudrate_bt
    def connect(self):
        self.connectZ()
        self.connectXY()
    def connectXY(self):
        self.serialDevice = serial.Serial(port=self.comport_pic, baudrate=self.baudrate_pic, timeout=1)
        self.serialDevice.dtr = 1
        self.serialDevice.rts = 1
        time.sleep(0.1)  # Reset condition.
        self.serialDevice.dtr = 0
        self.serialDevice.rts = 0
        time.sleep(1)  # Reset operation.
    def connectZ(self):
        self.esp = serial.Serial(self.comport_bt)
    def ping(self, ser):
        test_data = random.randint(0, 256)
        packet = [0xFF, 0xFF, 3, 0x01, test_data]  # N=2 (Have 1 instruction)
        ser.write(packet)
        time.sleep(0.1)
    def set_homeXY(self):
        packet = [255, 255, 5]
        while len(packet) != 30: packet.append(0)
        self.serialDevice.write(packet)
        time.sleep(0.1)
        self.x, self.y = 0, 0
    def set_homeZ(self):
        packet = [0xFF, 0xFF, 3, 0x05, 0]
        apply_checksum(packet)
        self.esp.write(packet)
        time.sleep(0.1)
        self.z = 400
        self.a = int(STEP_PER_ROUND*40 + (STEP_PER_ROUND/2))
    def readPosition(self):
        packet = [255, 255, 2]
        print("Reading")
        while len(packet) != 30: packet.append(0)
        self.serialDevice.write(packet)
        while True:
            if robot_event['readedXY']:
                packet = packet_readXY
                print("Read XY : " + str(packet))
                robot_event['readedXY'] = 0
                break
            time.sleep(0.1)
        print(int(packet[2]) * 256 + int(packet[3]), int(packet[4]) * 255 + int(packet[5]))
        return int(packet[2]) * 256 + int(packet[3]), int(packet[4]) * 255 + int(packet[5])  # X, Y
    def writeTrajectoryXY(self, x1, y1):
        print("==================================================================")
        print("writeTrajectoryXY - X: " + str(x1) + " Y: " +str(y1))
        x0, y0 = self.readPosition()
        print("X0=" + str(x0) + ", Y0=" + str(y0))
        c3, c4, theta, gramma, t = self.tj_solve(x1, y1, 5, x0, y0, 20)
        c3_packet = [0 if c3 > 0 else 1, int(abs(c3)) % 256, int((abs(c3) * 100) % 100), int((abs(c3) * 10000) % 100)]
        c4_packet = [0 if c4 > 0 else 1, int(abs(c4)) % 256, int((abs(c4) * 100) % 100), int((abs(c4) * 10000) % 100)]
        theta_packet = [0 if theta > 0 else 1, int(abs(theta)) % 256, int((abs(theta) * 100) % 100),
                        int((abs(theta) * 10000) % 100)]
        gramma_packet = [0 if gramma > 0 else 1, int(abs(gramma)) % 256, int((abs(gramma) * 100) % 100),
                         int((abs(gramma) * 10000) % 100)]
        t_packet = [int(t) % 256, int((t * 100) % 100), int((t * 10000) % 100)]
        packet = [255, 255, 0x04] + c3_packet + c4_packet + theta_packet + gramma_packet + t_packet
        while len(packet) != 30: packet.append(0)
        self.serialDevice.write(packet)
        robot_event['trajectoryXY_arrived'] = 0
        robot_event['trajectoryXY_running'] = 1
        print("PIC: " + str(packet))
        return t
    def writeTrajectoryZ(self, z1, a1):
        if z1 > 400: z1 = 400
        print("==================================================================")
        print("writeTrajectoryZ - Z: " + str(z1))
        ## A ##
        t=0
        packet = [0xFF, 0xFF, 7, 0x03, 0x01, int(a1 / 256), int(a1 % 256), int(t * 1000 / 256), int(t * 1000) % 256]
        apply_checksum(packet)
        self.esp.write(packet)
        x0, y0, x1, y1 = 100, 100, 105, 105
        z0 = self.z
        c3, c4, theta, gramma, t = self.tj_solve(x1, y1, z1, x0, y0, z0)
        c3_packet = [0 if c3 > 0 else 1, int(abs(c3)) % 256, int((abs(c3) * 100) % 100), int((abs(c3) * 10000) % 100)]
        c4_packet = [0 if c4 > 0 else 1, int(abs(c4)) % 256, int((abs(c4) * 100) % 100), int((abs(c4) * 10000) % 100)]
        theta_packet = [0 if theta > 0 else 1, int(abs(theta)) % 256, int((abs(theta) * 100) % 100),
                        int((abs(theta) * 10000) % 100)]
        gramma_packet = [0 if gramma > 0 else 1, int(abs(gramma)) % 256, int((abs(gramma) * 100) % 100),
                         int((abs(gramma) * 10000) % 100)]
        t_packet = [int(t) % 256, int((t * 100) % 100), int((t * 10000) % 100)]
        ## Z ## {255, 255, 16, 4, c3_0, c3_1, c3_2, c3_3, c4_0, c4_1, c4_2, c4_3, gamma_0, gamma_1, gamma_2, gamma_3, t_1, t_2, t_3, checkSum} // t = time in ms.
        packet = [0xFF, 0xFF, 17, 0x04] + c3_packet + c4_packet + gramma_packet + t_packet
        apply_checksum(packet)
        self.esp.write(packet)
        print("ESP_Z: " + str(packet))
        self.z = z1
        robot_event['trajectoryZ_arrived'] = 0
        robot_event['trajectoryZ_running'] = 1
        time.sleep(0.02)
        return t
    def writeTrajectory(self, x1, y1, z1, a1):  # Ack {255, 255, 3, 4, 1, 0} every move
        if z1 > 400: z1 = 400
        print("==================================================================")
        print("writeTrajectory - X: " + str(x1) + " Y: " + str(y1) + str(" Z: ") + str(z1))
        x0, y0 = self.readPosition()
        print("X0=" + str(x0) + ", Y0=" + str(y0) + ", X1=" + str(x1) + ", Y1=" + str(y1))
        z0 = self.z
        self.z = z1
        c3, c4, theta, gramma, t = self.tj_solve(x1, y1, z1, x0, y0, z0)
        c3_packet = [0 if c3 > 0 else 1, int(abs(c3)) % 256, int((abs(c3) * 100) % 100), int((abs(c3) * 10000) % 100)]
        c4_packet = [0 if c4 > 0 else 1, int(abs(c4)) % 256, int((abs(c4) * 100) % 100), int((abs(c4) * 10000) % 100)]
        theta_packet = [0 if theta > 0 else 1, int(abs(theta)) % 256, int((abs(theta) * 100) % 100), int((abs(theta) * 10000) % 100)]
        gramma_packet = [0 if gramma > 0 else 1, int(abs(gramma)) % 256, int((abs(gramma) * 100) % 100), int((abs(gramma) * 10000) % 100)]
        t_packet = [int(t) % 256, int((t * 100) % 100), int((t * 10000) % 100)]
        ## Z ## {255, 255, 16, 4, c3_0, c3_1, c3_2, c3_3, c4_0, c4_1, c4_2, c4_3, gamma_0, gamma_1, gamma_2, gamma_3, t_1, t_2, t_3, checkSum} // t = time in ms.
        packet = [0xFF, 0xFF, 17, 0x04] + c3_packet + c4_packet + gramma_packet + t_packet
        apply_checksum(packet)
        self.esp.write(packet)
        print("ESP_Z: " + str(packet))
        time.sleep(0.02)
        ## A ##
        packet = [0xFF, 0xFF, 7, 0x03, 0x01, int(a1 / 256), int(a1 % 256), int(t * 1000 / 256), int(t * 1000) % 256]
        apply_checksum(packet)
        self.esp.write(packet)
        ## XY ##
        packet = [255, 255, 0x04] + c3_packet + c4_packet + theta_packet + gramma_packet + t_packet
        while len(packet) != 30: packet.append(0)
        self.serialDevice.write(packet)
        print("PIC: " + str(packet))
        robot_event['trajectoryXY_arrived'] = 0
        robot_event['trajectoryZ_arrived'] = 0
        robot_event['trajectoryXY_running'] = 1
        robot_event['trajectoryZ_running'] = 1
        # time.sleep(0.1)
        # responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())  # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        # if len(responsePacket) != 4: return False  # Doesn't receive acknowledge
        return t
    def gripper(self, angle):  # {255, 255, 3, 6, servoPos, checksum} 20=close, 170=open
        packet = [0xFF, 0xFF, 3, 6, angle]
        apply_checksum(packet)
        self.esp.write(packet)
    def circular_motion(self, n):  # Ack {255, 255, 3, 4, 1, 0} every move
        self.writeTrajectoryXY(10, 150)
        while (robot_event['trajectoryXY_running'] and not robot_event['trajectoryXY_arrived']):
            time.sleep(1.2)
        packet = [255, 255, 7, 0, n]
        while len(packet) != 30: packet.append(0)
        self.serialDevice.write(packet)
        count = 0
    def tj_solve(self, x1, y1, z1, x0, y0, z0):
        vel_x_max = 80
        vel_y_max = 80
        vel_z_max = 700 * 7 / 250
        vel_x = 200
        vel_y = 200
        vel_z = 100
        t = 0.002
        c1 = 0
        c2 = 0
        c3 = 0
        c4 = 0
        theta = 0
        gramma = 0
        while abs(vel_x) > vel_x_max or abs(vel_y) > vel_y_max or abs(vel_z) > vel_z_max or t < 1.2:
            qf = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
            A = np.array([[t ** 2, t ** 3], [2 * t, 3 * (t ** 2)]])
            B = np.array([[qf], [0]])
            X = np.dot(np.linalg.inv(A), B)
            c3 = X[0][0]
            c4 = X[1][0]
            theta = math.atan2((y1 - y0), (x1 - x0))
            gramma = math.atan2(z1 - z0, math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2))

            theta_t = c1 + c2 * (t / 2) + c3 * (t / 2) * (t / 2) + c4 * (t / 2) * (t / 2) * (t / 2)
            theta_dot_t = c2 + 2 * c3 * (t / 2) + 3 * c4 * (t / 2) * (t / 2)
            vel_x = (theta_dot_t) * math.cos(gramma) * math.cos(theta)
            vel_y = (theta_dot_t) * math.cos(gramma) * math.sin(theta)
            vel_z = (theta_dot_t) * math.sin(gramma)
            t = t + 0.002
        # print("c3 = : ", c3)
        # print("c4 = : ", c4)
        # print("theta = : ", theta)
        # print("gamma = : ", gramma)
        # print("vel X = : ", vel_x)
        print("t = : ", t)
        return c3, c4, theta, gramma, t

    def closeXY(self):
        self.serialDevice.close()
    def closeZ(self):
        self.esp.close()
    def close(self):
        self.closeXY()
        self.closeZ()
robot = Robot("COM8", 115200, "COM6", 115200)
def send(packet):
    out_string = ""
    for item in packet:
        out_string += str(item)
        out_string += ","
    out_string = out_string[:-1]
    out_string += '\n' # End of command (Unity use ReadLine())
    client.send(out_string.encode("utf-8"))
def loading(status):
    if status == 0: send([3, 1, 0])
    if status == 1: send([3, 1, 1])

def robot_reciever():
    global packet_readXY
    while True:
        datas = robot.serialDevice.readline().decode('ASCII')
        if datas != '':
            if 'home' in datas:
                robot_event['homedXY'] = 1
                print("XY: homed")
                if robot_event['trajectoryZ_arrived'] == 1:
                    send([3, 1, 2])  # Deactivate Load Animation
                    robot_event['trajectoryZ_arrived'] = 0
                else: robot_event['trajectoryXY_arrived'] = 1
            packet = datas.split(',')
            if packet[0] == '6' and packet[1] == '2': # Read position data
                packet_readXY = packet.copy()
                robot_event['readedXY'] = 1
            if packet[0] == '3' and packet[1] == '7' and packet[2] == '0': # Start Circle
                event['circular_running'] = True
                print("Start Circular")
            if packet[0] == '3' and packet[1] == '7' and packet[2] == '1': # Stop Circle
                event['circular_running'] = False
                print("Stop Circular")
            if packet[0] == '3' and packet[1] == '4' and packet[2] == '1':
                robot_event['trajectoryXY_arrived'] = 1
                robot_event['trajectoryXY_running'] = 0
                print("XY: trajectory arrived")
                send([3, 1, 2])  # Deactivate Load Animation
                robot_event['trajectoryXY_arrived'] = 0

        responsePacket = robot.esp.read(robot.esp.inWaiting()) # ESP32
        if responsePacket:
            packet = list(responsePacket)
            packets_esp.append(packet)
            if packet[2] == 3 and packet[3] == 5 and packet[4] == 1:
                robot_event['homedZ'] = 1
                print("Z: homed")
                if robot_event['trajectoryXY_arrived']:
                    send([3, 1, 2])  # Deactivate Load Animation
                    robot_event['trajectoryXY_arrived'] = 0
                else: robot_event['trajectoryZ_arrived'] = 1
            if packet[2] == 3 and packet[3] == 5 and packet[4] == 2:
                robot_event['trajectoryZ_arrived'] = 1
                robot_event['trajectoryZ_running'] = 0
                print("Z: trajectory arrived")

        time.sleep(0.2)
        # HOME          [0xFF, 0xFF, 3, 5, 0x01, checksum]
        # TRAJECTORY    [0xFF, 0xFF, 3, 4, 0x01, checksum]
###############
###  UNITY  ###
###############
def recieving():
    global state, previous_state, run
    global event, recon_mode, segment_mode, use_standby
    while True:
        data = client.recv(64)
        if not data: continue # Pass if there is no data
        # print(data) # RAW bytes from Unity (send via TCP)
        if data[0] == 0: # Empty
            pass
        if data[0] == 1: # Change state
            state = int(data[1])
            use_standby = True
            if previous_state != state:
                previous_state = state
                print("State: " + state_name[state])
        if data[0] == 2: # Change Parameter
            if data[1] == 0: # CUDA setup
                USE_CUDA = bool(data[2])
            if data[2] == 1: # Auto RUN setup
                AUTO_RUN = bool(data[2])
            if data[2] == 2 : # Circular Motion Round(s)
                CIRCULAR_ROUND = data[2]
        if data[0] == 3: # Event (Button event)
            if data[1] == 0: event['capture'] = 1 # Capture Button
            if data[1] == 1: event['circular'] = 1 # Circular Capture Button
            if data[1] == 2: # Connect to Serial
                if data[2] == 0: # Connect All
                    if event['connectXY'] == 0: robot.connectXY()
                    if event['connectZ'] == 0: robot.connectZ()
                    event['connectXY'], event['connectZ'], event['connect'] = 1, 1, 1
                if data[2] == 1: # Connect XY
                    if event['connectXY'] == 0: robot.connectXY()
                    event['connectXY'] = 1
                    if event['connectZ']: event['connect'] = 1
                if data[2] == 2: # Connect Z
                    if event['connectZ'] == 0: robot.connectZ()
                    event['connectZ'] = 1
                    if event['connectXY']: event['connect'] = 1

            if data[1] == 3: # Home
                if data[2] == 0: # Home All
                    send([3, 1, 3]) # Activate Load Animation
                    robot.set_homeZ()
                    # robot.set_homeXY()
                    if event['homeXY'] == 0: robot.set_homeXY()
                    else: robot.writeTrajectoryXY(10, 10)
                    event['home'] = 1
                    event['homeXY'] = 1
                    event['homeZ'] = 1
                    robot_event['homing'] = 1
                if data[2] == 1: # Home XY
                    send([3, 1, 3]) # Activate Load Animation
                    if event['connectXY'] == 0: print('Not Connected')
                    if event['homeXY'] == 0: robot.set_homeXY()
                    else: robot.writeTrajectoryXY(10, 10)
                    event['homeXY'] = 1
                if data[2] == 2: # Home Z
                    send([3, 1, 3]) # Activate Load Animation
                    robot.set_homeZ()
                if data[2] == 3: # Center XY
                    send([3, 1, 3])  # Activate Load Animation
                    if event['homeXY'] == 0:
                        event['homeXY'] = 1
                        robot.set_homeXY()
                    robot.writeTrajectoryXY(200, 200)
            if data[1] == 4: # WriteTrajectory {3, 4, x_high, x_low, y_high, y_low, z_high, z_low, a_high, a_low}
                x_pos = data[2]*256 + data[3]
                y_pos = data[4]*256 + data[5]
                z_pos = data[6]*256 + data[7]
                print(z_pos)
                a_pos = data[8]*256 + data[9]
                if robot.x == x_pos and robot.y == y_pos: # No move in X, Y
                    if robot.z != z_pos or robot.a != a_pos: robot.writeTrajectoryZ(z_pos, a_pos)
                else:
                    if robot.z != z_pos or robot.a != a_pos:
                        send([3, 1, 3])  # Activate Load Animation
                        robot.writeTrajectory(x_pos, y_pos, z_pos, a_pos)
                    else:
                        send([3, 1, 3])  # Activate Load Animation
                        robot.writeTrajectoryXY(x_pos, y_pos)
                robot.x = x_pos
                robot.y = y_pos
                robot.z = z_pos
                robot.a = a_pos
            if data[1] == 5:  # Run Button
                if data[2] == 3:
                    event['recon'] = 1 # Reconstruction
                    recon_mode = data[3] # Get reconstruction mode
                if data[2] == 4:
                    event['segment'] = 1 # Segmentation
                    segment_mode = data[4]  # Get segmentation mode
                if data[2] == 5: event['locateMarker'] = 1 # Locate Marker
                if data[2] == 6: event['generatePath'] = 1 # Generate Path
                if data[2] == 7: event['visualize'] = 1 # Visualize
            if data[1] == 6: # Gripper Control
                if data[2] == 0:
                    print("Gripper Open")
                    robot.gripper(170)
                if data[2] == 1:
                    robot.gripper(20)
                    print("Gripper Close")
            if data[1] == 7: # TJ manual
                x_pos = data[2]*256 + data[3]
                y_pos = data[4]*256 + data[5]
                z_pos = data[6]*256 + data[7]
                a_pos = data[8]*256 + data[9]
                if robot.x == x_pos and robot.y == y_pos:  # No move in X, Y
                    if robot.z != z_pos or robot.a != a_pos:
                        t = robot.writeTrajectoryZ(z_pos, a_pos)
                else:
                    if robot.z != z_pos or robot.a != a_pos:
                        send([3, 1, 3])  # Activate Load Animation
                        t = robot.writeTrajectory(x_pos, y_pos, z_pos, a_pos)
                    else:
                        send([3, 1, 3])  # Activate Load Animation
                        t = robot.writeTrajectoryXY(x_pos, y_pos)
                robot.x, robot.y, robot.z, robot.a = x_pos, y_pos, z_pos, a_pos


def mainThread():
    global state, run, frame, cast, use_standby, command_queue
    cast = VDO2Unity() ## init opengl must be in thread to be able to run
    cast.createChannel(1280, 720, '1')
    cast.createChannel(800, 800, '2')
    cast.createChannel(800, 800, '3')
    print("State: " + state_name[state])
    while True:
        ### Do State ###
        if state == 0: # Setting
            if use_standby:
                use_standby = False
                if cv2.imread("X:/final.png") is not None:
                    cast.send('3', cv2.imread("X:/final.png"))
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "NO DATA", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)
        if state == 1: # Capture Template
            frame = cap.read()
            alpha = 0.5
            # rect_mask = cv2.rectangle(np.ones_like(frame)*255, (860, 440), (1060, 640), (0, 0, 0), -1)
            rect_mask = cv2.rectangle(np.ones_like(frame) * 255, (760, 340), (1160, 740), (0, 0, 0), -1)
            cv2.addWeighted(rect_mask, alpha, frame, 1 - alpha,0, frame)
            cast.send('1', imutils.resize(frame, height=720))
            if event['capture']:
                event['capture'] = 0 # Clear flag
                captured_image['template_raw'] = frame
                captured_image['template'] = imutils.resize(frame[340:740, 760:1160], height=200)
                cv2.imwrite("X:/template.png", captured_image['template'])
                cast.send('3', imutils.resize(captured_image['template'], height=800))
                aruco = aruco_crop(frame)
                if aruco != 0:  # if workspace avaliable
                    captured_image['warped'] = aruco[0]
            if use_standby:
                use_standby = False
                captured_image['template'] = cv2.imread("X:/template.png")
                if captured_image['template'] is not None:
                    display_image = imutils.resize(captured_image['template'], height=800)
                    cast.send('3', display_image)
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "Segmentation", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)
        if state == 2: # Capture Workspace
            frame = cap.read()
            aruco = aruco_crop(frame)
            if aruco != 0:  # if workspace avaliable
                warped, valid_mask = aruco
                cast.send('3', warped)
            if event['circular_running']:
                out = cv2.VideoWriter('X:/out.avi', fourcc, 30.0, (1920, 1080))
                while event['circular_running']:  # Time required = time.sleep(8*n+8)
                    frame = cap.read()
                    out.write(frame)
                out.release()
                vdo_warped, vdo_mask = [], []
                cap_vdo = cv2.VideoCapture("X:/out.avi")
                ret = cap_vdo.read()[0]
                while ret:
                    ret, frame_vdo = cap_vdo.read()
                    try:
                        aruco = aruco_crop(frame_vdo)
                        if aruco != 0:
                            warped, valid_mask = aruco
                            cast.send('3', warped)
                            vdo_warped.append(warped)
                            vdo_mask.append(valid_mask)
                    except: continue
                np.save("X:/warped", vdo_warped)
                np.save("X:/mask", vdo_mask)
            if event['circular']: # Capture button triggered
                event['circular'] = 0 # Clear flag
                # Z-axis up -> XY-axis home -> XY-axis circular -> Capture to RAMDisk
                robot.circular_motion(CIRCULAR_ROUND)
        if state == 3: # Reconstruction (Get X:/final.png)
            if event['recon']: # Reconstruct button triggered
                # recon_mode = 6
                start = time.time()
                print("Start Reconstrunction Process")
                send([3, 1, 1])  # Activate Load Animation
                if recon_mode == 0: # Median with valid mask
                    recon_median.run(N=20)
                if recon_mode == 1: # Median with valid mask & Rail mask
                    recon_median.run_rail(N=20, iteration=5)
                if recon_mode == 2: # Median with valid mask & Inpainting by rail mask (Using built-in OpenCV inpainting)
                    pass
                if recon_mode == 3: # Median with valid mask & Inpainting by rail mask (Using GAN with Tensorflow)
                    pass
                if recon_mode == 4: # BG-subtractor-Stochastic
                    BGsub.run(use_random=True, visualize=True)
                if recon_mode == 5: # BG-subtractor-Serial
                    BGsub.run(use_random=False)
                if recon_mode == 6: # BG-subtractor-Stochastic (Frankenstein image)
                    frankenstein.run(N=20)
                    BGsub_frank.run(use_random=True)
                if recon_mode == 7: # BG-subtractor-Serial (Frankenstein image)
                    frankenstein.run(N=20)
                    BGsub_frank.run(use_random=False)
                print("Reconstruction Finished!")
                event['recon'] = 0 # Clear event flag
                send([3, 1, 0])  # Deactivate Load Animation
                captured_image['reconstructed'] = cv2.imread("X:/final.png")
                cast.send('3', captured_image['reconstructed'])
                print("Process use " + str(int(time.time() - start)) + "second.")
            if use_standby:
                use_standby = False
                captured_image['reconstructed'] = cv2.imread("X:/final.png")
                if captured_image['reconstructed'] is not None: cast.send('3', captured_image['reconstructed'])
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "Reconstruction", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)
        if state == 4: # Segmentation
            if event['segment']:
                print("Start segmentation")
                send([3, 1, 1])  # Activate Load Animation
                captured_image['reconstructed'] = cv2.imread("X:/final.png")
                if segment_mode == 0: # Auto clustering for creating mask and guide grabcut
                    print("Mode 1")
                    captured_image['segment'] = DBSCAN.cluster_grap(captured_image['reconstructed'], esp_value=0.7, min_samples=300, N=1000, visualize=False)
                if segment_mode == 1: # Grabcut using manual guide
                    print("Mode 2")
                    pass
                if segment_mode == 2: # Auto clustering by multi iteration
                    print("Mode 3")
                    captured_image['segment'] = DBSCAN.cluster(captured_image['reconstructed'], esp_value=0.7, scale=2, N=1000)

                cv2.imwrite("X:/segment.png", captured_image['segment'])
                display_image = cv2.cvtColor(captured_image['segment'], cv2.COLOR_GRAY2BGR)

                cast.send('3', display_image)
                send([3, 1, 0])  # Deactivate Load Animation
                event['segment'] = 0 # Clear event flag
                print("Segmentation Finished")
            if use_standby:
                use_standby = False
                captured_image['segment'] = cv2.imread("X:/segment.png", 0)
                if captured_image['segment'] is not None:
                    display_image = captured_image['segment']
                    cast.send('3', display_image)
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "Segmentation", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)
        if state == 5: # Locate Marker
            if event['locateMarker']:
                send([3, 1, 1])  # Activate Load Animation
                if captured_image['template'] is None: captured_image['template'] = cv2.imread("X:/template.png")
                captured_image['startMask'], start_center = search_marker.run(captured_image['reconstructed'], captured_image['template'], visualize=False)
                cv2.imwrite("X:/startMask.png", captured_image['startMask'])
                captured_image['chessMask'], chess_center = search_chess.run(captured_image['segment'])
                cv2.imwrite("X:/chessMask.png", captured_image['chessMask'])
                display_image = cv2.bitwise_or(captured_image['startMask'], captured_image['chessMask'])
                cast.send('3', display_image)
                event['locateMarker'] = 0 # Clear event flag
                send([3, 1, 0])  # Deactivate Load Animation
            if use_standby:
                use_standby = False
                if captured_image['startMask'] is None: captured_image['startMask'] = cv2.imread("X:/startMask.png", 0)
                if captured_image['chessMask'] is None: captured_image['chessMask'] = cv2.imread("X:/chessMask.png", 0)
                if captured_image['startMask'] is not None and captured_image['chessMask'] is not None:
                    display_image = cv2.bitwise_or(captured_image['startMask'], captured_image['chessMask'])
                    cast.send('3', display_image)
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "Locate Marker", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)
        if state == 6: # Generate Path
            if event['generatePath']:
                send([3, 1, 1])  # Activate Load Animation
                if captured_image['reconstructed'] is None: captured_image['reconstructed'] = cv2.imread("X:/final.png")
                if captured_image['segment'] is None: captured_image['segment'] = cv2.imread("X:/segment.png", 0)
                if captured_image['startMask'] is None: captured_image['startMask'] = cv2.imread("X:/startMask.png", 0)
                if captured_image['chessMask'] is None: captured_image['chessMask'] = cv2.imread("X:/chessMask.png", 0)
                path_mask_opening = bicorn.getPathMask(captured_image['segment'], captured_image['startMask'], captured_image['chessMask'])
                cast.send('3', cv2.cvtColor(path_mask_opening, cv2.COLOR_GRAY2BGR))

                path_cnts, path_cnts_approx, canvas = bicorn.getPath(path_mask_opening=path_mask_opening, visualize=False)
                cast.send('3', canvas)
                waypoints3D_approx_list = bicorn.getPath3D(src=captured_image['reconstructed'], path_cnts=path_cnts, path_cnts_approx=path_cnts_approx)
                command_queue, canvas = bicorn.generateCommand(captured_image['startMask'], captured_image['chessMask'], path_cnts, waypoints3D_approx_list)
                colorMap = colormap.get_colormap()
                canvas = cv2.bitwise_or(canvas, colorMap)
                cast.send('3', canvas)
                cv2.imwrite("X:/generatePath.png", canvas)
                print(command_queue)
                event['generatePath'] = 0 # Clear event flag
                send([3, 1, 0])  # Deactivate Load Animation
            if use_standby:
                use_standby = False
                captured_image['generatePath'] = cv2.imread("X:/generatePath.png")
                if captured_image['generatePath'] is not None:
                    alpha = 0.5
                    canvas = captured_image['generatePath'].copy()
                    captured_image['reconstructed'] = cv2.imread("X:/final.png")
                    original_mask = captured_image['reconstructed']
                    cv2.addWeighted(original_mask, alpha, canvas, 1 - alpha, 0, canvas)
                    cast.send('3', canvas)
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "Generate Path", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)
        if state == 7: # Visualize & Execute
            if event['visualize']:
                send([3, 1, 1])  # Activate Load Animation
                robot_event['trajectoryXY_arrived'] = 1
                robot_event['trajectoryZ_arrived'] = 1
                robot.gripper(170)
                ### Gripper on duty ###
                gripper_queue = [(69, 376, 200, robot.a), (69, 376, 65, robot.a), -1, (69, 376, 80, robot.a), (69, 365, 80, robot.a), (69, 365, 290, robot.a)] # (67, 200, 290, 400)
                while len(gripper_queue) > 0:
                    if gripper_queue[0] == -1:
                        gripper_queue.pop(0)
                        robot.gripper(20)
                        time.sleep(1)
                    else:
                        (x_pos, y_pos, z_pos, a_pos) = gripper_queue[0]
                        z_pos += z_offset
                        if robot.x == x_pos and robot.y == y_pos:  # No move in X, Y
                            if robot.z != z_pos or robot.a != a_pos:
                                t = robot.writeTrajectoryZ(z_pos, a_pos)
                        else:
                            if robot.z != z_pos or robot.a != a_pos:
                                t = robot.writeTrajectory(x_pos, y_pos, z_pos, a_pos)
                            else:
                                t = robot.writeTrajectoryXY(x_pos, y_pos)
                        robot.x, robot.y, robot.z, robot.a = x_pos, y_pos, z_pos, a_pos
                        gripper_queue.pop(0)
                        while (robot_event['trajectoryXY_running'] and not robot_event['trajectoryXY_arrived']) or (robot_event['trajectoryZ_running'] and not robot_event['trajectoryZ_arrived']):
                            time.sleep(0.5)
                print(command_queue)
                # command_queue = [(335.0, 65.0, 400), (335.0, 65.0, 200), (330.5, 145.0, 200), (327.5, 333.5, 100), (135.0, 334.5, 100), (60.04003700277521, 334.88940240518036, 100), (61.0, 300.5, 100), (63.5, 92.0, 200), (93.5, 101.5, 200), (170.0, 176.0, 200), (220.5, 223.0, 200), (220.5, 223.0, 400)]
                command_queue_bin = []
                translation = (+5, -5) # Correction of position
                trans_coff = 0.02

                while len(command_queue):
                    print("GO TO " + str(command_queue[0]))
                    (y_pos, x_pos, z_pos) = command_queue[0]
                    x_pos = 400 - x_pos
                    y_pos = 400 - y_pos
                    x_pos -= trans_coff * x_pos
                    y_pos -= trans_coff * y_pos
                    if x_pos < 0: x_pos = 0
                    if x_pos >= 400: x_pos = 399
                    if y_pos < 0: y_pos = 0
                    if y_pos >= 400: y_pos = 399
                    z_pos += z_offset
                    if z_pos > 350: z_pos = 350
                    # Find new angle
                    if len(command_queue) > 1 and len(command_queue_bin):
                        zeta = math.atan2(command_queue[0][1]-command_queue_bin[-1][1], command_queue[0][0]-command_queue_bin[-1][0])
                        # Find present angle & delta angle
                        zeta_now = (robot.a%800) * (math.pi/400)
                        if abs(zeta - zeta_now) <= math.pi:
                            zeta_delta = zeta - zeta_now
                        else: # Try to use shortest way
                            if zeta - zeta_now > 0:
                                zeta_delta = (zeta - zeta_now) - math.pi
                            else: zeta_delta = (zeta - zeta_now) + math.pi
                        if zeta_delta < -math.pi/2: zeta_delta = math.pi+zeta_delta
                        if zeta_delta > math.pi/2: zeta_delta = zeta_delta-math.pi
                        print("Zeta delta: " + str(zeta_delta))
                        a_delta = int(zeta_delta * (400/math.pi))
                        a_pos = robot.a + a_delta
                    else: a_pos = robot.a
                    while a_pos < 0: a_pos += 800 # Avoid alpha < 0 (not supported in protocol)
                    print("Alpha: " + str(a_pos))
                    x_pos, y_pos, z_pos = int(x_pos), int(y_pos), int(z_pos)
                    if robot.x == x_pos and robot.y == y_pos:  # No move in X, Y
                        if robot.z != z_pos or robot.a != a_pos:
                            t = robot.writeTrajectoryZ(z_pos, a_pos)
                    else:
                        if robot.z != z_pos or robot.a != a_pos:
                            t = robot.writeTrajectory(x_pos, y_pos, z_pos, a_pos)
                        else:
                            t = robot.writeTrajectoryXY(x_pos, y_pos)
                    robot.x, robot.y, robot.z, robot.a = x_pos, y_pos, z_pos, a_pos
                    command_queue_bin.append(command_queue[0])
                    command_queue.pop(0)
                    while (robot_event['trajectoryXY_running'] and not robot_event['trajectoryXY_arrived']) or (robot_event['trajectoryZ_running'] and not robot_event['trajectoryZ_arrived']):
                        time.sleep(1)
                command_queue = command_queue_bin.copy()
                send([3, 1, 0])  # Deactivate Load Animation
                event['visualize'] = 0 # Clear event flag
            if use_standby:
                use_standby = False
                captured_image['generatePath'] = cv2.imread("X:/generatePath.png")
                if captured_image['generatePath'] is not None:
                    alpha = 0.5
                    canvas = captured_image['generatePath'].copy()
                    captured_image['reconstructed'] = cv2.imread("X:/final.png")

                    original_mask = captured_image['reconstructed']

                    cv2.addWeighted(original_mask, alpha, canvas, 1 - alpha, 0, canvas)
                    cast.send('3', canvas)
                else:
                    standby_image1 = standby_image.copy()
                    cv2.putText(standby_image1, "Generate Path", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255))
                    cast.send('3', standby_image1)

if __name__ == "__main__":
    t1 = threading.Thread(target=mainThread, name='t1')   # creating threads
    t2 = threading.Thread(target=recieving, name='t2')
    t1.start()  # starting threads
    t2.start()
    while True:
        time.sleep(1)
        if event['connectXY'] == 1:
            t3 = threading.Thread(target=robot_reciever, name='t3')
            t3.start()
            break