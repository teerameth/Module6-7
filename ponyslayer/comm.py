import serial
import random
import time
import numpy as np
import math
import serial.tools.list_ports
comport_pic = "COM4"
baudrate_pic = 115200
comport_bt = "COM6"
baudrate_bt = 115200
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
        self.serialDevice = serial.Serial(port = self.comport_pic, baudrate = self.baudrate_pic, timeout=1)
        self.serialDevice.dtr = 1
        self.serialDevice.rts = 1
        time.sleep(0.1) # Reset condition.
        self.serialDevice.dtr = 0
        self.serialDevice.rts = 0
        time.sleep(1) # Reset operation.
    def connectZ(self):
        self.esp = serial.Serial(self.comport_bt)
    def ping(self, ser):
        test_data = random.randint(0, 256)
        packet = [0xFF, 0xFF, 3, 0x01, test_data] # N=2 (Have 1 instruction)
        apply_checksum(packet)
        ser.write(packet)
        time.sleep(0.1)
        responsePacket = ser.read(ser.inWaiting())
        print(responsePacket)
        if packet == [int(x) for x in responsePacket]: return True
        return False
    def set_homeXY(self):
        packet = [0xFF, 0xFF, 3, 0x05, 0]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        time.sleep(0.1)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting()) # [0xFF, 0xFF, 5, 0x01, checksum]
        print(responsePacket)
    def set_homeZ(self):
        packet = [0xFF, 0xFF, 3, 0x05, 0]
        apply_checksum(packet)
        self.esp.write(packet)
        time.sleep(0.1)
        responsePacket = self.esp.read(self.esp.inWaiting()) # [0xFF, 0xFF, 5, 0x01, checksum]
        print(responsePacket)
    def readPosition(self):
        packet = [0xFF, 0xFF, 2, 0x02]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        time.sleep(0.1)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x02, X_high, X_low, Y_high, Y_low, checksum]
        print(responsePacket)
        return responsePacket[4]*256+responsePacket[5], responsePacket[6]*255+responsePacket[7] # X, Y
    def writePositionXY(self, x, y):
        packet = [0xFF, 0xFF, 6, 0x03, int(x/256), x%256, int(y/256), y%256]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        time.sleep(0.1)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        if len(responsePacket) != 4: return False # Doesn't receive acknowledge
        return True
    def writePositionZ(self, z, a):
        ## Z ## {255, 255, 5, 3, 0, high_byte, low_byte, checkSum}
        packet = [0xFF, 0xFF, 5, 0x00, int(z/256), z%256]
        apply_checksum(packet)
        self.esp.write(packet)
        print(packet)
        ## A ## {255, 255, 5, 3, 1, high_byte, low_byte, checkSum}
        packet = [0xFF, 0xFF, 6, 0x03, int(a/256), a%256]
        apply_checksum(packet)
        self.esp.write(packet)
        print(packet)
    def writeTrajectory(self, t, x1, y1, z1 ,a1): # Ack {255, 255, 3, 4, 1, 0} every move
        x0, y0 = self.readPosition()
        z0, z1 = 0, 0
        c3,c4,theta,gramma = self.tj_solve(t, x1, y1, z1, x0, y0, z0)
        c3_packet = [0 if c3>0 else 1, int(abs(c3))%256, int((abs(c3)*100)%100), int((abs(c3)*10000)%100)]
        c4_packet = [0 if c4>0 else 1, int(abs(c4))%256, int((abs(c4)*100)%100), int((abs(c4)*10000)%100)]
        theta_packet = [0 if theta>0 else 1, int(abs(theta))%256, int((abs(theta)*100)%100), int((abs(theta)*10000)%100)]
        gramma_packet = [0 if gramma>0 else 1, int(abs(gramma))%256, int((abs(gramma)*100)%100), int((abs(gramma)*10000)%100)]
        t_packet = [int(t)%256, int((t*100)%100), int((t*10000)%100)]
        packet = [0xFF, 0xFF, 21, 0x04] + c3_packet + c4_packet + theta_packet + gramma_packet + t_packet
        apply_checksum(packet)
        self.serialDevice.write(packet)
        print("PIC: " + str(packet))
        ## Z ## {255, 255, 8, 3, 0, high_byte, low_byte, t_high, t_low, checkSum} // t = time in ms.
        packet = [0xFF, 0xFF, 8, 0x03, 0x00, int(z1/256), int(z1%256), int(t*1000/256), int(t*1000)%256]
        apply_checksum(packet)
        self.esp.write(packet)
        ## A ## {255, 255, 8, 3, 1, high_byte, low_byte, t_high, t_low, checkSum} // t = time in ms.
        packet = [0xFF, 0xFF, 8, 0x03, 0x00, int(a1/256), int(a1%256), int(t*1000/256), int(t*1000)%256]
        apply_checksum(packet)
        self.esp.write(packet)

        time.sleep(0.1)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        if len(responsePacket) != 4: return False # Doesn't receive acknowledge
        return True
    def gripper(self, angle): # {255, 255, 3, 6, servoPos, checksum}
        packet = [0xFF, 0xFF, 3, 6, angle]
        apply_checksum(packet)
        self.esp.write(packet)
    def circular_motion(self, n): # Ack {255, 255, 3, 4, 1, 0} every move
        packet = [0xFF, 0xFF, 4, 0x07, 0x00, n]
        apply_checksum(packet)
        self.serialDevice.write(packet)
        count = 0
        while True:
            responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())
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
    def closeXY(self): self.serialDevice.close()
    def closeZ(self): self.esp.close()
    def close(self):
        self.closeXY()
        self.closeZ()
def main():
    robot = Robot("COM4", 115200, "COM6", 115200)
    robot.connectZ()
    robot.ping(robot.esp)
    robot.gripper(120)
    robot.set_homeZ()
if __name__ == '__main__':
    main()