'''
Description: 
    Uses serial.write() to transmit string terminated with '\n'
    Uses serial.read_until() to receive message terminated with '\n'
    Program terminates
'''
import serial
from time import sleep

portName = '/dev/ttyACM0'
baudRate = 38400

try:
    ''' 
    Construct a serial object with default arguments
    bytesize 8 bits
    parity   None
    stopbits 1
    timeout  None   (read timeout)
    write_timeout None
    '''
    ser = serial.Serial(portName, baudRate)
    print ("opened port " + ser.name)

except serial.SerialException:
        raise IOError("Problem connecting to " + portName)

# opening serial connection resets Arduino, give it some time to reset
sleep(2)

# flush input buffer, discarding all contents
ser.reset_input_buffer()

sendMsg = 'hello\n'.encode('utf-8')

'''
write(data)
Parameters: data - data to send
Returns: number of bytes written.
Return type: int

Write the bytes data to the port. data should be of type bytes or bytearray.
Unicode strings must be encoded (e.g. 'hello',encode('utf-8'))
'''
print('transmitting: ', end ='')
print(sendMsg)
bytesWritten = ser.write(sendMsg)

print('bytesWritten: ', end='')
print(bytesWritten)

# sit here until bytes are received
while ser.in_waiting < 1:
    pass

bytesRead = ser.read_until()
print('bytesRead: ', end='')
print(bytesRead)


ser.close()
print("closed serial connection, program finished")
    