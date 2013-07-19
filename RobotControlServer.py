import socket
import select
import sys

host = ''
port = int(raw_input("Enter the port to listen on: "))
backlog = 5
size = 1

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((host, port))

#Listen for incoming connection on the port given by user
s.listen(backlog)

conn, addr = s.accept()
print "Connection from ", addr

while 1:
	data = conn.recv(size)
	if not data:
		break
	print "Receieved: ", data
	#conn.send(data)

conn.close()
