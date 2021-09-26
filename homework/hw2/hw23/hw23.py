'''
Homework 2,  Problem  3

Description: Extracts x, y values from serial data read

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
        # expecting 4 bytes total
        if ser.in_waiting > 3:
            # read will block until 2 bytes are read
            readByte = ser.read(2) 

            # big endian byte order means the most significant byte contains the most signifcant bits
            x = int.from_bytes(readByte, byteorder='little',signed=False)
           
            readByte = ser.read(2)
            y = int.from_bytes(readByte, byteorder='little',signed=False)
            
            print("x: {:x}, y: {:x}".format(x,y))

        # kill a bit of time before running through loop again
        # Arduino program transmitting every 300 ms
        sleep(50/1000)

    print('while loop terminated')
    ser.close()
    print("closed port")



