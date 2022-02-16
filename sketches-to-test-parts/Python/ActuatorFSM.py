from multiprocessing.connection import wait
from numpy import mean

# bound = 40
state = [0,0,0,0,0]
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
    global state
    tsign = getDir(trans)
    psign = getDir(pitch)
    ysign = getDir(yaw)
    rsign = getDir(roll)
    if mean(trans) > tbound:
        state[0] = state[0] + 1
    else:
        state[0] = state[0] - 1
        if (trans[-1]-trans(0))/(tstamps[-1]-tstamps[0]) > vthresh:
            state[1] = state[1] + 1
        else:
            state[0] = state[0] - 1
            if mean(pitch) > pbound or mean(yaw) > ybound:
                if mean(pitch) > pbound:
                    state[2] = state[2] + 1
                else:
                    state[2] = state[2] - 1
                if mean(yaw) > ybound:
                    state[3] = state[3] + 1
                else:
                    state[3] = state[3] - 1
            else: 
                if mean(roll) > rbound:
                    state[4] = state[4] + 1
                else:
                    state[4] = state[4] - 1
    match state.index:
        case 0:
            withdraw_sr(tsign)
            state[0] = 0
        case 1:
            accel_sr(tsign)
            state[1] = 0
        case 2:
            adjust_pitch(psign)
            state[2] = 0
        case 3:
            adjust_yaw(ysign)
            state[3] = 0
        case 4: 
            adjust_roll(rsign)
            state[4] = 0
    wait(2000)
