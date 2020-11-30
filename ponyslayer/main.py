import cv2, numpy, time, os, threading, socket, imutils, serial, math
from unicorn2 import *
from transform import *
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
event = {'capture':0, 'circular':0, 'connect':0, 'connectXY':0, 'connectZ':0, 'home':0, 'homeXY':0, 'homeZ':0, 'circular_running':0}
robot_event = {'homing':0, 'homingXY':0, 'homedXY':0, 'homingZ':0, 'homedZ':0, 'trajectoryXY_running':0, 'trajectoryXY_arrived':0, 'readedXY':0}
captured_image = {'warped':None, 'template_raw':None, 'template':None, 'reconstructed':None}
fourcc = cv2.VideoWriter_fourcc(*'XVID')
fourcc1 = cv2.VideoWriter_fourcc(*'XVID')
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
        apply_checksum(packet)
        ser.write(packet)
        time.sleep(0.1)
    def set_homeXY(self):
        packet = [0xFF, 0xFF, 3, 0x05, 0]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        time.sleep(0.1)
    def set_homeZ(self):
        self.Z = 400
        packet = [0xFF, 0xFF, 3, 0x05, 0]
        apply_checksum(packet)
        self.esp.write(packet)
        time.sleep(0.1)
    def readPosition(self):
        packet = [0xFF, 0xFF, 2, 0x02]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        # print(packet)
        time.sleep(0.1)
        while True:
            if robot_event['readedXY']:
                packet = packets[-1]
                robot_event['readedXY'] = 0
                break
        return packet[4] * 256 + packet[5], packet[6] * 255 + packet[7]  # X, Y
    def writePositionXY(self, x, y):
        packet = [0xFF, 0xFF, 6, 0x03, int(x / 256), x % 256, int(y / 256), y % 256]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        time.sleep(0.1)
        responsePacket = self.serialDevice.read(
            self.serialDevice.inWaiting())  # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        if len(responsePacket) != 4: return False  # Doesn't receive acknowledge
        return True
    def writePositionZ(self, z, a):
        self.Z = z
        ## Z ## {255, 255, 5, 3, 0, high_byte, low_byte, checkSum}
        packet = [0xFF, 0xFF, 5, 0x03, 0, int(z / 256), z % 256]
        apply_checksum(packet)
        self.esp.write(packet)
        print(packet)
        time.sleep(0.2)
        ## A ## {255, 255, 5, 3, 1, high_byte, low_byte, checkSum}
        packet = [0xFF, 0xFF, 5, 0x03, 1, int(a / 256), a % 256]
        apply_checksum(packet)
        self.esp.write(packet)
        print(packet)
    def writeTrajectoryXY(self, t, x1, y1):
        x0, y0 = self.readPosition()
        print("X0=" + str(x0) + ", Y0=" + str(y0))
        c3, c4, theta, gramma = self.tj_solve(t, x1, y1, 0, x0, y0, 0)
        c3_packet = [0 if c3 > 0 else 1, int(abs(c3)) % 256, int((abs(c3) * 100) % 100), int((abs(c3) * 10000) % 100)]
        c4_packet = [0 if c4 > 0 else 1, int(abs(c4)) % 256, int((abs(c4) * 100) % 100), int((abs(c4) * 10000) % 100)]
        theta_packet = [0 if theta > 0 else 1, int(abs(theta)) % 256, int((abs(theta) * 100) % 100),
                        int((abs(theta) * 10000) % 100)]
        gramma_packet = [0 if gramma > 0 else 1, int(abs(gramma)) % 256, int((abs(gramma) * 100) % 100),
                         int((abs(gramma) * 10000) % 100)]
        t_packet = [int(t) % 256, int((t * 100) % 100), int((t * 10000) % 100)]
        packet = [0xFF, 0xFF, 21, 0x04] + c3_packet + c4_packet + theta_packet + gramma_packet + t_packet
        apply_checksum(packet)
        self.serialDevice.write(packet)
        print("PIC: " + str(packet))
    def writeTrajectoryZ(self, t, z1, a1):
        ## A ##
        packet = [0xFF, 0xFF, 7, 0x03, 0x01, int(a1 / 256), int(a1 % 256), int(t * 1000 / 256), int(t * 1000) % 256]
        apply_checksum(packet)
        self.esp.write(packet)
        x0, y0, x1, y1 = 0, 0, 0, 0
        z0 = self.Z
        c3, c4, theta, gramma = self.tj_solve(t, x1, y1, z1, x0, y0, z0)
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
        self.Z = z1
        time.sleep(0.02)
    def writeTrajectory(self, t, x1, y1, z1, a1):  # Ack {255, 255, 3, 4, 1, 0} every move
        x0, y0 = self.readPosition()
        print("X0=" + str(x0) + "Y0=" + str(y0))
        z0 = self.Z
        self.Z = z1
        c3, c4, theta, gramma = self.tj_solve(t, x1, y1, z1, x0, y0, z0)
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
        time.sleep(0.02)
        ## A ##
        packet = [0xFF, 0xFF, 7, 0x03, 0x01, int(a1 / 256), int(a1 % 256), int(t * 1000 / 256), int(t * 1000) % 256]
        apply_checksum(packet)
        self.esp.write(packet)
        ## XY ##
        packet = [0xFF, 0xFF, 21, 0x04] + c3_packet + c4_packet + theta_packet + gramma_packet + t_packet
        apply_checksum(packet)
        self.serialDevice.write(packet)
        print("PIC: " + str(packet))

        time.sleep(0.1)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())  # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        if len(responsePacket) != 4: return False  # Doesn't receive acknowledge
        return True
    def writeLinearZ(self, t, z1, a1):
        self.Z = z1
        ## Z ## {255, 255, 8, 3, 2, high_byte, low_byte, t_high, t_low, checkSum} // t = time in ms.
        packet = [0xFF, 0xFF, 7, 0x03, 0x02, int(z1 / 256), int(z1 % 256), int(t * 1000 / 256), int(t * 1000) % 256]
        apply_checksum(packet)
        print(packet)
        self.esp.write(packet)
        time.sleep(0.01)
        ## A ## {255, 255, 8, 3, 3, high_byte, low_byte, t_high, t_low, checkSum} // t = time in ms.
        t_esp = t * 0.85
        packet = [0xFF, 0xFF, 7, 0x03, 0x03, int(a1 / 256), int(a1 % 256), int(t_esp * 1000 / 256), int(t_esp * 1000) % 256]
        apply_checksum(packet)
        print(packet)
        self.esp.write(packet)
    def gripper(self, angle):  # {255, 255, 3, 6, servoPos, checksum} 20=close, 170=open
        packet = [0xFF, 0xFF, 3, 6, angle]
        apply_checksum(packet)
        self.esp.write(packet)
    def circular_motion(self, n):  # Ack {255, 255, 3, 4, 1, 0} every move
        packet = [0xFF, 0xFF, 4, 0x07, 0x00, n]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        count = 0
    def tj_solve(self, t, x1, y1, z1, x0, y0, z0):
        # print(t, x1, y1, z1, x0, y0, z0)
        qf = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
        A = np.array([[t ** 2, t ** 3], [2 * t, 3 * (t ** 2)]])
        B = np.array([[qf], [0]])
        X = np.dot(np.linalg.inv(A), B)
        theta = math.atan2((y1 - y0), (x1 - x0))
        gramma = math.atan2(z1 - z0, math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2))
        c3 = X[0][0]
        c4 = X[1][0]
        # print("c3 = " + str(c3) + "\nc4 = " + str(c4) + "\ntheta = " + str(theta) + "\ngramma = " + str(gramma))
        return c3, c4, theta, gramma
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

packets = []
packets_esp = []
def robot_reciever():
    while True:
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # PIC
        if responsePacket:
            packet = list(responsePacket)
            packets.append(packet)
            if packet[2] == 3 and packet[3] == 5 and packet[4] == 1:
                robot_event['homedXY'] = 1
                send([3, 1, 2])  # Deactivate Load Animation
                packets.pop(-1)
            if packet[2] == 3 and packet[3] == 4 and packet[4] == 1:
                robot_event['trajectoryXY_arrived'] = 1
                send([3, 1, 2])  # Deactivate Load Animation
                packets.pop(-1)
            if packet[2] == 6 and packet[3] == 2:
                robot_event['readedXY'] = 1
            if packet[2] == 3 and packet[3] == 7 and packet[4] == 0: # Start Circle
                event['circular_running'] = True
                packets.pop(-1)
            if packet[2] == 3 and packet[3] == 7 and packet[4] == 1: # Stop Circle
                event['circular_running'] = False
                packets.pop(-1)
            print(packets)
            print("Recieved: " + str(packet))

        responsePacket = robot.esp.read(robot.esp.inWaiting()) # ESP32
        if responsePacket:
            packet = list(responsePacket)
            packets_esp.append(packet)
            if packet[2] == 3 and packet[3] == 5 and packet[4] == 1:
                robot_event['homedZ'] = 1
                send([3, 1, 2])  # Deactivate Load Animation
        time.sleep(0.2)
        # HOME          [0xFF, 0xFF, 3, 5, 0x01, checksum]
        # TRAJECTORY    [0xFF, 0xFF, 3, 4, 0x01, checksum]
def recieving():
    global state, previous_state, run
    global event
    while True:
        data = client.recv(64)
        if not data: continue # Pass if there is no data
        print(data)
        if data[0] == 0: # Empty
            pass
        if data[0] == 1: # Change state
            state = int(data[1])
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
                    event['connectXY'], event['connectZ']= 1, 1
                if data[2] == 1: # Connect XY
                    if event['connectXY'] == 0: robot.connectXY()
                    event['connectXY'] = 1
                if data[2] == 2: # Connect Z
                    if event['connectZ'] == 0: robot.connectZ()
                    event['connectZ'] = 1
            if data[1] == 3: # Home
                if data[2] == 0: # Home All
                    send([3, 1, 3]) # Activate Load Animation
                    robot.set_homeZ()
                    robot.set_homeXY()
                    event['home'] = 1
                    event['homeXY'] = 1
                    event['homeZ'] = 1
                    robot_event['homing'] = 1
                if data[2] == 1: # Home XY
                    send([3, 1, 3]) # Activate Load Animation
                    if event['connectXY'] == 0: print('Not Connected')
                    if event['homeXY'] == 0: robot.set_homeXY()
                    else: robot.writeTrajectoryXY(5, 10, 10)
                    event['homeXY'] = 1
                if data[2] == 2: # Home Z
                    send([3, 1, 3]) # Activate Load Animation
                    robot.set_homeZ()
                if data[2] == 3: # Center XY
                    send([3, 1, 3])  # Activate Load Animation
                    if event['homeXY'] == 0:
                        event['homeXY'] = 1
                        robot.set_homeXY()
                    robot.writeTrajectoryXY(5, 200, 200)
            if data[1] == 4: # WriteTrajectory {3, 4, x_high, x_low, y_high, y_low, z_high, z_low, a_high, a_low}
                x_pos = data[2]*256 + data[3]
                y_pos = data[4]*256 + data[5]
                z_pos = data[6]*256 + data[7]
                a_pos = data[8]*256 + data[9]
                robot.writeTrajectory(5, x_pos, y_pos, z_pos, a_pos)



def mainThread():
    global state, run, frame, cast
    cast = VDO2Unity() ## init opengl must be in thread to be able to run
    cast.createChannel(1280, 720, '1')
    cast.createChannel(800, 800, '2')
    cast.createChannel(800, 800, '3')
    print("State: " + state_name[state])
    while True:
        ### Do State ###
        if state == 0: # Setting
            if captured_image['warped'] is not None:
                cast.send('2', captured_image['warped'])
        if state == 1: # Capture Template
            frame = cap.read()
            cast.send('1', imutils.resize(frame, height=720))
            if event['capture']:
                event['capture'] = 0 # Clear flag
                captured_image['template_raw'] = frame
                aruco = aruco_crop(frame)
                if aruco != 0:  # if workspace avaliable
                    captured_image['warped'] = aruco[0]
                cv2.imshow("Template_RAW", captured_image['template_raw'])
                cv2.waitKey(1)
        if state == 2: # Capture Workspace
            frame = cap.read()
            aruco = aruco_crop(frame)
            if aruco != 0:  # if workspace avaliable
                warped, valid_mask = aruco
                cast.send('2', warped)
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
                            cast.send('2', warped)
                            vdo_warped.append(warped)
                            vdo_mask.append(valid_mask)
                    except: continue
                np.save("X:/warped", vdo_warped)
                np.save("X:/mask", vdo_mask)
            if event['circular']: # Capture button triggered
                event['circular'] = 0 # Clear flag
                # Z-axis up -> XY-axis home -> XY-axis circular -> Capture to RAMDisk
                robot.circular_motion(CIRCULAR_ROUND)
                pass
        if state == 3: # Reconstruction
            pass
        if state == 4: # Segmentation
            pass
        if state == 5: # Locate Marker
            pass
        if state == 6: # Generate Path
            pass
        if state == 7: # Visualize
            pass

if __name__ == "__main__":
    t1 = threading.Thread(target=mainThread, name='t1')   # creating threads
    t2 = threading.Thread(target=recieving, name='t2')
    t1.start()  # starting threads
    t2.start()
    while True:
        if event['connectXY'] == 1:
            t3 = threading.Thread(target=robot_reciever, name='t3')
            t3.start()
            break

