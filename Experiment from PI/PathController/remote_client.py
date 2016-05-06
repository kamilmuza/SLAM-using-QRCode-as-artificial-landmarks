import socket

IP = '10.0.0.1'
PORT = 50007
addr = (IP, PORT)

message = """ Aqui escribimos luego, pero no puede faltar Danilo y Tavito y darle el berro a Silvio """

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(addr)

print(message)

while True:
	command = input('>>> ')
	if command == 'end':
		break
	s.send(command.encode()) 
s.close()