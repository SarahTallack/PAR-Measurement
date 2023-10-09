import serial
import time

timestamp = time.strftime('%M') + "_AS7341.csv"
# f = open("YELLOW_10.csv", "w+")

# colour = ["RED", "YELLOW", "GREEN", "CYAN", "BLUE", "MAGENTA", "WHITE"]
# filename = 

def readserial(comport, baudrate, timestamp=False):

    ser = serial.Serial(comport, baudrate, timeout=0.1)         # 1/timeout is the frequency at which the port is read

    # if timestamp:
    #     f.write(f'Time,Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7,Channel8,Clear,NIR\n')
    # else:
    #     f.write(f'Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7,Channel8,Clear,NIR\n')

    i = 0
    while i<20:

        data = ser.readline().decode().strip()

        if data and timestamp:
            timestamp = time.strftime('%H:%M:%S')
            f.write(f'{timestamp},{data} \n')
            print(i)
            i = i + 1
        elif data:
            f.write(f'{data} \n')
            print(i)
            i = i + 1


# def createfile(name):

#     filename = name + ".txt"
#     f = open(filename, "w+")

colour = ["RED", "YELLOW", "GREEN", "CYAN", "BLUE", "MAGENTA", "WHITE"]
colour_basic = ["RED", "GREEN", "BLUE", "WHITE"]

# TEST 1
# for col in colour:
#     print(col)
#     for brightness in range(10):
#         filename = col + "_" + str((brightness+1)*10) +".csv"
#         print(filename)
#         f = open(filename, "w+")
#         readserial('COM7', 115200, False)
#         f.close()

# TEST 2
# for col in colour_basic:
#     print(col)
#     for brightness in range(10):
#         filename = "WATER_" + col + "_" + str((brightness+1)*10) +".csv"
#         print(filename)
#         f = open(filename, "w+")
#         readserial('COM7', 115200, False)
#         f.close()

# TEST 3
for r in range(3, 0, -1):
    for c in range(3, 0, -1):
        filename = "WHITE_50_R" + str(r) + "_C" + str(c) +".csv"
        print(filename)
        f = open(filename, "w+")
        readserial('COM7', 115200, False)
        f.close()

# TEST 1
for col in colour:
    print(col)
    for brightness in range(10):
        filename = col + "_" + str((brightness+1)*10) +".csv"
        print(filename)
        f = open(filename, "w+")
        readserial('COM7', 115200, False)
        f.close()

# readserial('COM7', 115200, False)
# f.close()
print("done")