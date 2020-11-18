import serial
# from serial.tools.list_ports import comports
import random
import time
import numpy as np
import math

class Robot:
    def __init__(self, com, baud):
        self.str_comport = com
        self.str_baudrate = baud
    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.str_baudrate, timeout=1)
        self.serialDevice.dtr = 1
        self.serialDevice.rts = 1
        time.sleep(0.1) # The time required for the device to recognize the reset condition.
        self.serialDevice.dtr = 0
        self.serialDevice.rts = 0
        time.sleep(1) # The time required for the device to complete the reset operation.
    def apply_checksum(self, l):
        checkSumOrdList = l[2:]
        checkSumOrdListSum = sum(checkSumOrdList)
        computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
        l.append(computedCheckSum)
    def ping(self):
        test_data = random.randint(0, 256)
        packet = [0xFF, 0xFF, 3, 0x01, test_data] # N=2 (Have 1 instruction)
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        time.sleep(0.02)
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting())
        print(responsePacket)
        if packet == [int(x) for x in responsePacket]: return True
        return False
    def set_home(self):
        packet = [0xFF, 0xFF, 3, 0x05, 0]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # [0xFF, 0xFF, 5, 0x01, checksum]
        print(responsePacket)
    def readPosition(self):
        packet = [0xFF, 0xFF, 2, 0x02]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        time.sleep(0.02)
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x02, X_high, X_low, Y_high, Y_low, checksum]
        print(responsePacket)
        return responsePacket[4]*256+responsePacket[5], responsePacket[6]*255+responsePacket[7] # X, Y
    def writePosition(self, x, y):
        packet = [0xFF, 0xFF, 6, 0x03, int(x/256), x%256, int(y/256), y%256]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        time.sleep(0.02)
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        if len(responsePacket) != 4: return False # Doesn't receive acknowledge
        return True
    def writeTrajectory(self, t, x1, y1): # Ack {255, 255, 3, 4, 1, 0} every move
        x0, y0 = self.readPosition()
        z0, z1 = 0, 0
        c3,c4,theta,gramma = self.tj_solve(t, x1, y1, z1, x0, y0, z0)
        c3_packet = [0 if c3>0 else 1, int(abs(c3))%256, int((abs(c3)*100)%100), int((abs(c3)*10000)%100)]
        c4_packet = [0 if c4>0 else 1, int(abs(c4))%256, int((abs(c4)*100)%100), int((abs(c4)*10000)%100)]
        theta_packet = [0 if theta>0 else 1, int(abs(theta))%256, int((abs(theta)*100)%100), int((abs(theta)*10000)%100)]
        gramma_packet = [0 if gramma>0 else 1, int(abs(gramma))%256, int((abs(gramma)*100)%100), int((abs(gramma)*10000)%100)]
        t_packet = [int(t)%256, int((t*100)%100), int((t*10000)%100)]
        packet = [0xFF, 0xFF, 21, 0x04] + c3_packet + c4_packet + theta_packet + gramma_packet + t_packet
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        time.sleep(0.02)
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        if len(responsePacket) != 4: return False # Doesn't receive acknowledge
        return True
    def circular_motion(self, n): # Ack {255, 255, 3, 4, 1, 0} every move
        packet = [0xFF, 0xFF, 4, 0x07, 0x00, n]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        count = 0
        while True:
            responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting())
            if len(responsePacket) == 6: count+=1
            if count == 4*n: break # Wait for complete
    def tj_solve(self, t, x1, y1, z1, x0, y0, z0):
        print(t, x1, y1, z1, x0, y0, z0)
        qf = math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)
        A = np.array([[t**2,t**3],[2*t,3*(t**2)]])
        B = np.array([[qf],[0]])
        X = np.dot(np.linalg.inv(A),B)
        theta = math.atan2((y1-y0),(x1-x0))
        gramma = math.atan2(z1-z0,math.sqrt((x1-x0)**2 + (y1-y0)**2))
        c3 = X[0][0]
        c4 = X[1][0]
        print("c3 = " + str(c3) + "\nc4 = " + str(c4) + "\ntheta = " + str(theta) + "\ngramma = " + str(gramma))
        return c3,c4,theta,gramma

robot = Robot("COM4", 115200)
robot.connect()

while True:
    print("Trying to connect")
    if robot.ping():
        print("Connected")
        break
    time.sleep(1)
robot.set_home()
robot.circular_motion(1)
robot.writePosition(200, 250)
robot.writePosition(200, 200)
robot.writePosition(150, 150)
robot.writePosition(100, 100)
x, y = robot.readPosition()
packet = [255, 255, 3, 5, 0]
robot.apply_checksum(packet)
print(packet)
robot.readPosition()
robot.writeTrajectory(3, 100, 100)
robot.writeTrajectory(3, 300, 300)
robot.writeTrajectory(3, 200, 100)
robot.writeTrajectory(3, 100, 200)