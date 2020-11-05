import urllib.request
root_url = "http://192.168.43.140"
def sendRequest(url):
    n = urllib.request.urlopen(url)
while True:
    answer = input("input: ")
    if answer == "on":
        sendRequest(root_url+"/H")
    if answer == "off":
        sendRequest(root_url+"/L")
    if answer == "up":
        sendRequest(root_url+"/U")
    if answer == "down":
        sendRequest(root_url+"/D")
    if answer == "cw":
        sendRequest(root_url+"/CW")
    if answer == "ccw":
        sendRequest(root_url+"/CCW")
    else:
        sendRequest(root_url+"/?value=" + answer + "&")