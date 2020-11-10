import serial
# from serial.tools.list_ports import comports
import random
import time

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
        packet = [0xFF, 0xFF, 2, 0x01] # N=2 (Have 0 instruction)
        self.apply_checksum(packet)
        print(packet)
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
    def readPosition(self):
        packet = [0xFF, 0xFF, 3, 0x02]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        time.sleep(0.02)
        responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x02, X, Y, checksum]
        print(responsePacket)
        return responsePacket[4], responsePacket[5] # X, Y
    def writePosition(self, x, y):
        packet = [0xFF, 0xFF, 6, 0x03, int(x/256), x%256, int(y/256), y%256]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)
        print(packet)
        # time.sleep(0.02)
        # responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting()) # [0xFF, 0xFF, 3, 0x03, 0] = Acknowledge, [0xFF, 0xFF, 3, 0x03, 1] = Arrived
        # if len(responsePacket) != 4: return False # Doesn't receive acknowledge
        # return True
    def circular_motion(self, n):
        packet = [0xFF, 0xFF, 4, 0x07, 0x00, n]
        self.apply_checksum(packet)
        self.serialDevice.write(packet)

robot = Robot("COM5", 115200)
robot.connect()

# while True:
#     print("Trying to connect")
#     if robot.ping():
#         print("Connected")
#         break
#     time.sleep(1)
robot.set_home()
robot.writePosition(200, 250)
# x, y = robot.readPosition()
# packet = [255, 255, 3, 5, 0]
# robot.apply_checksum(packet)
# print(packet)
