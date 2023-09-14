import serial
import time

timestamp = time.strftime('%M') + "_AS7341.csv"
f = open(timestamp, "w+")


def readserial(comport, baudrate, timestamp=False):

    ser = serial.Serial(comport, baudrate, timeout=0.1)         # 1/timeout is the frequency at which the port is read

    # if timestamp:
    #     f.write(f'Time,Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7,Channel8,Clear,NIR\n')
    # else:
    #     f.write(f'Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7,Channel8,Clear,NIR\n')

    i = 0
    while i<10:

        data = ser.readline().decode().strip()

        if data and timestamp:
            timestamp = time.strftime('%H:%M:%S')
            f.write(f'{timestamp},{data} \n')
            print(i)
            i = i + 1
        elif data:
            f.write(data)


# def createfile(name):

#     filename = name + ".txt"
#     f = open(filename, "w+")


readserial('COM3', 115200, True)
f.close()
print("done")