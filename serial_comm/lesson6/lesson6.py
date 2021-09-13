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

    portName = "/dev/ttyACM1"
    ser = serialConnect(portName)
    count = 0

    while keepRunning == True:

        value = 512
        value_b = value.to_bytes(2, 'big')
        ser.write(value_b)
        # print to see how data is encoded
        print("value_b", end='')
        print(value_b)

        while ser.in_waiting < 1:
            pass 
        print("called read_until")
        bytesRead = ser.read_until()
        print(bytesRead)
        
        count += 1
        # kill a bit of time before running through loop again
        # Arduino program transmitting every 2000 ms
        sleep(500/1000)

    print('while loop terminated')
    ser.close()
    print("closed port")