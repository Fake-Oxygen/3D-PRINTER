import serial, time
import io

stm = serial.Serial('COM6', 9600, timeout=.5)
time.sleep(1)


file = open('E:/3D-PRINTER/SOFTWARE/example.gcode', 'r')
content = file.readlines()

stm.write(str.encode(line))
count = 0
for line in content:
    count += 1
    print("Line{}: {}".format(count, line.strip()))
    stm.write(str.encode(line))
    
    while True:
        data = stm.readline()
        print(data)
        if 'Done' in data.decode('utf-8'):
            # print(data)

            # time.sleep(1)
            break