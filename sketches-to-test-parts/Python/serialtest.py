import serial

ser = serial.Serial(
    port='COM3',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)
count=1

def isData(line):
   return any(char.isdigit() for char in line)
def splitData(line):
    temp = line.decode().split(';')
    for i in range(0,len(temp)):
        temp[i] = temp[i].split('|')
    for i in range(0,len(temp)):
        for j in range(0,len(temp[i])):
            temp[i][j] = temp[i][j].split(',')
    return temp

while True:
    line = ser.readline()
    if(isData(str(line))):
        line = splitData(line)
        print(line)
        print()
    count = count+1

ser.close()