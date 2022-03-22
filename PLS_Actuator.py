# from multiprocessing.connection import wait
import numpy as np
import serial


def windowData():
    win_trans_r = win_pitch_r = win_yaw_r = win_roll_r = win_trans_l = win_pitch_l = win_yaw_l = win_roll_l = []
    i = 0
    while i < 3:
        data = rl.readline().decode()
        line = data.strip('\r\n').split(", ")
        print(line)
        if len(line) > 20:
            win_trans_r.append(float(line[17]))
            print(win_trans_r)
            win_pitch_r.append(float(line[1]))
            win_yaw_r.append(float(line[2]))
            win_roll_r.append(float(line[16]))
            win_trans_l.append(float(line[20]))
            win_pitch_l.append(float(line[4]))
            win_yaw_l.append(float(line[5]))
            win_roll_l.append(float(line[19]))
            i = i + 1
    return [win_trans_r, win_pitch_r, win_yaw_r, win_roll_r, win_trans_l, win_pitch_l, win_yaw_l, win_roll_l]


def getDir(win):
    print(win)
    if win[-1] - win[0] < 0:
        return 1
    else:
        return 0


def actuatorFSM(trans, pitch, yaw, roll, right):
    pbound = tbound = rbound = ybound = 1
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
    if np.mean(trans) > tbound:
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
        if np.mean(pitch) > pbound or np.mean(yaw) > ybound:
            if np.mean(pitch) > pbound:
                state[1] = state[1] + 1
                if state[1] == 4:
                    if state[2] == 4:
                        state[2] = state[2] - 1
                    if state[2] == 4:
                        state[3] = state[3] - 1
            else:
                state[1] = state[1] - 1
            if np.mean(yaw) > ybound:
                state[2] = state[2] + 1
                if state[2] == 4:
                    if state[3] == 4:
                        state[3] = state[3] - 1
            else:
                state[2] = state[2] - 1
        else:
            if np.mean(roll) > rbound:
                state[3] = state[3] + 1
            else:
                state[3] = state[3] - 1
    try:
        sr = state.index(4)
    except:
        ser.write("ok\r".encode())
        # print('All good!')
        # print(state)
    else:
        if sr == 0:
            state[0] = state[0] - 1
            triggerSR('w', tsign, right)
        elif sr == 1:
            state[1] = state[1] - 1
            triggerSR('p', psign, right)
        elif sr == 2:
            state[2] = state[2] - 1
            triggerSR('y', ysign, right)
        elif sr == 3:
            state[3] = state[3] - 1
            triggerSR('r', rsign, right)
    if right == 1:
        stater = state
    else:
        statel = state


def triggerSR(sr, sign, lr):
    if sr == 'w':
        print("w" + str(sign) + str(lr) + "\r")
        ser.write(("w" + str(sign) + str(lr) + "\r").encode())
    elif sr == 's':
        print("s" + str(sign) + str(lr) + "\r")
        ser.write(("s" + str(sign) + str(lr) + "\r").encode())
    elif sr == 'p':
        print("p" + str(sign) + str(lr) + "\r")
        ser.write(("p" + str(sign) + str(lr) + "\r").encode())
    elif sr == 'y':
        print("y" + str(sign) + str(lr) + "\r")
        ser.write(("y" + str(sign) + str(lr) + "\r").encode())
    elif sr == 'r':
        print("r" + str(sign) + str(lr) + "\r")
        ser.write(("r" + str(sign) + str(lr) + "\r").encode())


stater = [0, 0, 0, 0]
statel = [0, 0, 0, 0]

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
        win = windowData()
        print(win)
        actuatorFSM(win[0], win[1], win[2], win[3], 0)
        actuatorFSM(win[4], win[5], win[6], win[7], 1)
    except KeyboardInterrupt:
        ser.close()
        print("ctrl-c quit")
