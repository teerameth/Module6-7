import cv2, numpy, time, os, threading, socket, imutils
from unicorn2 import *
from comm import *
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
event = {'capture':0, 'circular':0, 'connect':0, 'connectXY':0, 'connectZ':0}
captured_image = {'template_raw':None, 'template':None, 'reconstructed':None}
robot = Robot("COM8", 115200, "COM12", 115200)
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
                    send([3, 1, 2]) # Deactivate Load Animation
                if data[2] == 1: # Home XY
                    send([3, 1, 3]) # Activate Load Animation
                    robot.set_homeXY()
                    send([3, 1, 2]) # Deactivate Load Animation
                if data[2] == 2: # Home Z
                    send([3, 1, 3]) # Activate Load Animation
                    robot.set_homeZ()
                    send([3, 1, 2]) # Deactivate Load Animation


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
            pass
        if state == 1: # Capture Template
            frame = cap.read()
            cast.send('1', imutils.resize(frame, height=720))
            if event['capture']:
                event['capture'] = 0 # Clear flag
                captured_image['template_raw'] = frame
                cv2.imshow("Template_RAW", captured_image['template_raw'])
                cv2.waitKey(1)
        if state == 2: # Capture Workspace
            frame = cap.read()
            if event['circular']: # Capture button triggered
                event['circular'] = 0 # Clear flag
                # Z-axis up -> XY-axis home -> XY-axis circular -> Capture to RAMDisk
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
    t1.join()    # wait until all threads finish
    t2.join()
    while True:
        cast.send('1', imutils.resize(frame, height=720))