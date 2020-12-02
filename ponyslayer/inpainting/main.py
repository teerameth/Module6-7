import cv2, numpy, time, os, threading, socket, imutils, serial, math
backlog = 1
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Waiting for Main")
s.connect(('127.0.0.1', 11111))
print("Connected")
def send(packet):
    out_string = ""
    for item in packet:
        out_string += str(item)
        out_string += ","
    out_string = out_string[:-1]
    out_string += '\n' # End of command (Unity use ReadLine())
    s.sendall(out_string.encode("utf-8"))

def reciever():
    send([255, 255, 1, 2, 3])
    time.sleep(1)
def mainThread():
    pass
if __name__ == "__main__":
    t1 = threading.Thread(target=mainThread, name='t1')   # creating threads
    t2 = threading.Thread(target=reciever, name='t2')
    t1.start()
    t2.start()
