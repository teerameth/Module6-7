import urllib.request
root_url = "http://192.168.0.136"
def sendRequest(url):
    n = urllib.request.urlopen(url)
while True:
    answer = input("input: ")
    if answer == "on":
        sendRequest(root_url+"/H")
    if answer == "off":
        sendRequest(root_url+"/L")
    