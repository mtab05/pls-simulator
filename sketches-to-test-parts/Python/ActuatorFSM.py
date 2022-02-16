from numpy import mean


global state
state = [0,0,0,0]
def windowData(line):
    tstamps = win_trans = win_pitch = win_yaw = win_roll = []
    for i in range(0,5,len(line)):
        tstamps[i] = line[i]
        win_trans[i] = line[i+1]
        win_pitch[i] = line[i+2]
        win_yaw[i] = line[i+3]
        win_roll[i] = line[i+4]
    return [tstamps,win_trans,win_pitch,win_yaw,win_roll]

def getDir(win):
    if(win[-1]-win[0]  < 0):
        return 1
    else:
        return 0

def actuatorFSM(tstamps,trans,pitch,yaw,roll):
    tsign = getDir(trans)
    psign = getDir(pitch)
    ysign = getDir(yaw)
    rsign = getDir(roll)
    match state:
        case [0,0,0,0]:
            state = []