'''
    Receives serial data from Arduino.
    Assumes device path is /dev/ttyACM0. Change this to match your system's path.
    Program loops until 5 bytes have been read, then terminates.
    The byte data read is printed.
'''
import serial
from time import sleep


# change the path name to match your system
portName = '/dev/ttyACM0'

try:
    ''' 
    Construct a serial object with default arguments
    baudrate 9600
    bytesize 8 bits
    parity   None
    stopbits 1
    timeout  None   (read timeout)
    '''
    ser = serial.Serial(portName)
    print ("opened port " + ser.name + '\n')

except serial.SerialException:
        raise IOError("Problem connecting to " + portName)

# sleep short time to ensure port is open
sleep(50/1000)

# flush input buffer, discarding all contents
ser.reset_input_buffer()


# loop a few times to illustrate data received
count = 0
while count < 5:
    # don't try to read unless there are bytes in the receive buffer
    if ser.in_waiting > 0:
        ''' read_until(expected=LF,size=None)

        read until an expected sequence is found('\n' by default),
        the size is exceeded or until timeout occurs. With no timeout
        it will block until the requested number of bytes is read
        '''
        bytesRead = ser.read_until()

        # convert byte string to unicode string, remove leading and trailing whitespace
        serialStr = bytesRead.decode().strip()   # decode default is 'utf-8'

        # convert byte string to int, remove leading and trailing whitespace
        intVal = int(bytesRead.strip())


        # print data types just once
        if count == 0:
            print("Data Types")
            print("bytesRead: ", end='')
            print(type(bytesRead))
            print("serialStr: ", end='')
            print(type(serialStr))
            print("intVal: ", end='')
            print(type(intVal))
            print('')

        # print data in its various forms
        print('bytesRead: ' + str(bytesRead))
        print('serialStr: ' + serialStr)
        print('intVal:    ', end='')
        print(intVal)
        print('')

        # to illustrate the byte array contains the ASCII value of each character
        print("Printing integer values of byte array elements")
        for b in bytesRead:
            print(b, end=' ')

        print('\n')
        
        count += 1

ser.close()