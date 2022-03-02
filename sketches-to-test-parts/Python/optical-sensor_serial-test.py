import serial
import matplotlib.pyplot as plt

ser = serial.Serial(
    port='COM6',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0)

print("connected to: " + ser.portstr)
count = 1


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
x_opt, y_opt, samples = [], [], []
# Create figure for plotting
fig = plt.figure(figsize=(8, 6))
ax = fig.subplots(2)
fig.subplots_adjust(hspace=0.8)
sample_number = 0

while True:
    line = rl.readline().decode()
    data = line.strip('\r\n').split()
    print(data)
    if len(data) == 3:
        sample_number = int(data[2])
        x_opt.append(int(data[0]))
        y_opt.append(int(data[1]))
        samples.append(int(data[2]))
    #if sample_number > 200:
        #break

# Clear plot of previous iteration and plot collected accelerometer data of current iteration
ax[0].clear()
ax[0].plot(samples, x_opt, c='blue')
ax[0].set_title("Optical x Signal")
ax[0].set_xlabel("Samples")
#ax[0].set_ylim([-30, 30])

# Clear plot of previous iteration and plot collected gyroscope data of current iteration
ax[1].clear()
ax[1].plot(samples, y_opt, c='green')
ax[1].set_title("Optical y Signal")
ax[1].set_xlabel("Samples")
#ax[1].set_ylim([-30, 30])

plt.show()

