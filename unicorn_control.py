import serial
import random

class Robot:
    def __init__(self, com, baud):
        self.str_comport = com
        self.str_baudrate = baud
    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.str_baudrate, timeout=0)
    def ping(self):
        test_data = random.randint(0, 256)
        packet = [0xFF, 0xFF, 3, 0x01, test_data]
        checkSumOrdList = packet[2:]
        checkSumOrdListSum = sum(checkSumOrdList)
        computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
        packet.append(computedCheckSum)
        self.serialDevice.write(packet)
robot = Robot("COM16", 115200)
robot.connect()
robot.ping()
responsePacket = robot.serialDevice.read(robot.serialDevice.inWaiting())
print(responsePacket)