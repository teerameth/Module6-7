import time
import threading
def test1():
    for i in range(5):
        time.sleep(1)
        print("Printing, test1:"+str(i))
def test2():
    for i in range(5):
        time.sleep(2)
        print("Printing, test2:"+str(i))
x = threading.Thread(target=test1)
t = threading.Thread(target=test2)
x.start()
t.start()