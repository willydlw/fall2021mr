'''
Lesson 5 Description: Extracts x, y values from serial data read

Uses CTRL+C signal handler to terminate while loop execution

'''
import serial
from signal import signal, SIGINT
from time import sleep

keepRunning = True 

def handler(signal, frame):
    global keepRunning 
    print('SIGINT or CTRL+C detected, setting keepRunning to False')
    keepRunning = False


def serialConnect(portName):
    try: 
        ser = serial.Serial(portName)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)




if __name__ == '__main__':

    #register the signal handler
    signal(SIGINT, handler)

    portName = "/dev/ttyACM0"
    ser = serialConnect(portName)
    

    while keepRunning == True:

        if ser.in_waiting > 0:
            bytesRead = ser.read_until()
            xylist = [int(v) for v in bytesRead.decode().split(':')]
            if len(xylist) == 2: 
                x = xylist[0]
                y = xylist[1]
                print("x: " + str(x) + ", y: " + str(y))
            else:
                print("warning xylist length is " + len(xylist))

        # kill a bit of time before running through loop again
        # Arduino program transmitting every 300 ms
        sleep(50/1000)

    print('while loop terminated')
    ser.close()
    print("closed port")



