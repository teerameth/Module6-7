import socket, time
backlog = 1
size = 64
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('127.0.0.1', 12345))
s.listen(backlog)
print("is waiting")
client, address = s.accept()

while 1:
    data = client.recv(size)
    if data:
        print(data)
    client.send("1,2,3,4\n".encode("utf-8"))
    time.sleep(0.2)

# try:
#     print("is waiting")
#     client, address = s.accept()
#
#     while 1:
#         data = client.recv(size)
#         if data:
#             print(data)
#         client.send(bytes("1,2,3,4\n"))
#         time.sleep(0.2)
#
# except:
#     print("closing socket")
#     client.close()
#     s.close()