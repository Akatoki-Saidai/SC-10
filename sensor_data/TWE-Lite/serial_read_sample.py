import  serial 

s = serial.Serial("/dev/serial0",115200,timeout=10)
c = s.read()
s = s.read(10)
l = s.readline()
print(c)
print(s)
print(l)
s.close()
