# from multiprocessing.connection import wait
from numpy import mean
import serial
import time
      
def windowData(line,right=0):
    win_trans = win_pitch = win_yaw = win_roll = []
    for i in range(0,4,len(line)):
        win_trans[i] = line[i]
        win_pitch[i] = line[i+1]
        win_yaw[i] = line[i+2]
        win_roll[i] = line[i+3]
    return [win_trans,win_pitch,win_yaw,win_roll,right]

def getDir(win):
    if(win[-1]-win[0]  < 0):
        return 1
    else:
        return 0

def actuatorFSM(trans,pitch,yaw,roll,right):
    if right == 1:
        global stater
        state = stater
    else:
        global statel
        state = statel
    tsign = getDir(trans)
    psign = getDir(pitch)
    ysign = getDir(yaw)
    rsign = getDir(roll)
    if mean(trans) > tbound:
        state[0] = state[0] + 1
        if state[0] == 4:
            if state[1] == 4:
                state[1] = state[1] - 1
            if state[2] == 4:
                state[2] = state[2] - 1
            if state[3] == 4:
                state[3] = state[3] - 1
    else:
            state[0] = state[0] - 1
            if mean(pitch) > pbound or mean(yaw) > ybound:
                if mean(pitch) > pbound:
                    state[1] = state[1] + 1
                    if state[1] == 4:
                        if state[2] == 4:
                            state[2] = state[2] - 1
                        if state[2] == 4:
                            state[3] = state[3] - 1
                else:
                    state[1] = state[1] - 1
                if mean(yaw) > ybound:
                    state[2] = state[2] + 1
                    if state[2] == 4:
                        if state[3] == 4:
                            state[3] = state[3] - 1
                else:
                    state[2] = state[2] - 1
            else: 
                if mean(roll) > rbound:
                    state[3] = state[3] + 1
                else:
                    state[3] = state[3] - 1
    if right == 1:
        stater = state
    else:
        statel = state
    match state.index(4):
        case 0:
            triggerSR('w',tsign,right)
        case 1:
            triggerSR('s',tsign,right)
        case 2:
            triggerSR('p',psign,right)
        case 3:
            triggerSR('y',ysign,right)
        case 4: 
            triggerSR('r',rsign,right)

def triggerSR(sr,sign,lr):
    match sr:
        case 'w':
            ser.write(("w"+str(sign)+str(lr)+":\n").encode())
        case 's':
            ser.write(("s"+str(sign)+str(lr)+":\n").encode())
        case 'p':
            ser.write(("p"+str(sign)+str(lr)+":\n").encode())
        case 'y':
            ser.write(("y"+str(sign)+str(lr)+":\n").encode())
        case 'r':
            ser.write(("r"+str(sign)+str(lr)+":\n").encode())

stater = [0,0,0,0,0]
statel = [0,0,0,0,0]

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0)

print("connected to: " + ser.portstr)

# https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                return r
            else:
                self.buf.extend(data)

rl = ReadLine(ser)
while True:
    try:
        line = rl.readline().decode()
        data = line.strip('\r\n').split(", ")
        win = windowData(data)
        actuatorFSM(win[0],win[1],win[2],win[3],win[4],win[5])
    except KeyboardInterrupt:
        ser.close()
        print("ctrl-c quit")