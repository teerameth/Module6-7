# import bluetooth
# devices = bluetooth.discover_devices(lookup_names=True)
# print(type(devices))
# print("Devices found: %s" % len(devices))
# for item in devices:
#     print(item)
def apply_checksum(l):
    checkSumOrdList = l[2:]
    checkSumOrdListSum = sum(checkSumOrdList)
    computedCheckSum = ( ~(checkSumOrdListSum%256) ) % 256
    l.append(computedCheckSum)
    return l
import serial
import serial.tools.list_ports
com_port = ""
ports = list(serial.tools.list_ports.comports())
for p in ports:
    if "Bluetooth" in p[1]:
        com_port = p[0]
        break
print(com_port)
ser = serial.Serial(com_port)  # open serial port
ser.write([255, 255, 2, 1, 252])
ser.close()             # close port