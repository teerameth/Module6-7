import cv2, numpy, time, os, threading, socket, imutils
from unicorn2 import *
from comm import *
backlog = 1
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('127.0.0.1', 12345))
s.listen(backlog)
print("is waiting")
client, address = s.accept()
state_name = ['setting', 'captureTemplate', 'captureWorkspace', 'reconstruction', 'segmentation']
### Parameters ###
USE_CUDA = True
cap = Camera()
run, state, previous_state = 1, 1, 1
event = {'capture':0, 'circular':0}
captured_image = {'template_raw':None, 'template':None, 'reconstructed':None}

def send(packet):
    out_string = ""
    for item in packet:
        out_string += str(item)
        out_string += ","
    out_string = out_string[:-1]
    out_string += '\n' # End of command (Unity use ReadLine())
    client.send(out_string.encode("utf-8"))
def recieving():
    global state, previous_state, run
    global event
    while True:
        data = client.recv(64)
        if not data: continue # Pass if there is no data
        if data[0] == 0: # Empty
            pass
        if data[0] == 1: # Change state
            state = data[1]
            if previous_state != state:
                previous_state = state
                print("State: " + state_name[state])
        if data[0] == 2: # Change Parameter
            pass
        if data[0] == 3: # Event (Button event)
            if data[1] == 0: event['capture'] = 1 # Capture Button
            if data[1] == 1: event['circular'] = 1 # Circular Capture Button
def mainThread():
    global state, run, frame, cast
    cast = VDO2Unity() ## init opengl must be in thread to be able to run
    cast.createChannel(1280, 720, '1')
    cast.createChannel(800, 800, '2')
    cast.createChannel(800, 800, '3')

    while True:
        ### Do State ###
        if state == 0: # Setting
            pass
        if state == 1: # Capture Template
            frame = cap.read()
            cast.send('1', imutils.resize(frame, height=720))
            if event['capture']:
                event['capture'] = 0
                captured_image['template_raw'] = frame
                cv2.imshow("Template_RAW", captured_image['template_raw'])
                cv2.waitKey(1)
        if state == 2: # Capture Workspace
            frame = cap.read()
            pass
        if state == 3: # Reconstruction
            pass
        if state == 4: # Segmentation
            pass

if __name__ == "__main__":
    t1 = threading.Thread(target=mainThread, name='t1')   # creating threads
    t2 = threading.Thread(target=recieving, name='t2')
    t1.start()  # starting threads
    t2.start()
    t1.join()    # wait until all threads finish
    t2.join()
    while True:
        cast.send('1', imutils.resize(frame, height=720))