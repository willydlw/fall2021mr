'''
Description: illlustrates using serial read function to read a single byte value.

The byte read is printed in hexadecimal format.
'''
import serial
from time import sleep

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
    print ("opened port " + ser.name)

except serial.SerialException:
        raise IOError("Problem connecting to " + portName)

# sleep short time to ensure port is open
sleep(50/1000)

# flush input buffer, discarding all contents
ser.reset_input_buffer()

count = 0
while count < 10:
    # only read when a byte is available
    if(ser.inWaiting() > 0):
        '''
        read(size=1)   
        size parameter is number of bytes to read
        returns bytes read from the port
        return type: bytes

        If a timeout is set it may return less characters as requested. 
        With no timeout it will block until the requested number of bytes is read.
        '''
        readByte = ser.read()
        
        print("byte format: ", end = " ")
        print(readByte)
        # big endian byte order means the most significant byte contains the most signifcant bits
        print("int format:  ", int.from_bytes(readByte, byteorder='big',signed=False))
        print('')  
        count += 1

ser.close()

