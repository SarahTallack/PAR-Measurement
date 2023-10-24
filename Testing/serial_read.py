import serial
import time
import winsound

timestamp = time.strftime('%M') + "_AS7341.csv"


# Set frequency to 2000 Hertz
frequency = 1000
# Set duration to 1500 milliseconds (1.5 seconds)
duration = 750
winsound.Beep(frequency, duration)
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
    while i<60:

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
# for col in colour_basic:
#     print(col)
#     for brightness in range(1):
#         filename = col + "_100_VEML_SINGLE" + ".csv"
#         print(filename)
#         f = open(filename, "w+")
#         readserial('COM6', 115200, False)
#         f.close()
#         winsound.Beep(frequency, duration)

# TEST 2
for col in colour_basic:
    print(col)
    for brightness in range(10):
        filename = "PLASTIC_" + col + "_" + str((brightness+1)*10) +".csv"
        print(filename)
        f = open(filename, "w+")
        readserial('COM6', 115200, False)
        f.close()
        # Make beep sound on Windows
        winsound.Beep(frequency, duration) 

# TEST 3
# for r in range(2):
#     filename = "WHITE_100_Further_R" + str(r+1) + ".csv"
#     print(filename)
#     f = open(filename, "w+")
#     readserial('COM7', 115200, False)
#     f.close()
#     # Make beep sound on Windows
#     winsound.Beep(frequency, duration)

# TEST 4
# for d in range(6):
#     filename = "VEML_WHITE_100_Dist" + str(d) + ".csv"
#     print(filename)
#     f = open(filename, "w+")
#     readserial('COM4', 115200, False)
#     f.close()
#     # Make beep sound on Windows
#     winsound.Beep(frequency, duration)

winsound.Beep(frequency, duration)
# readserial('COM7', 115200, False)
# f.close()
print("done")

#COM7 on PC, COM4 on laptop