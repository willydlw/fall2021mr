'''
Description: 
   Receives serial data containing accelerometer and gyroscope
   sensor data.
   
   Extracts x, y, z values 

   Writes data to file after receiving a fixed number of samples

Uses CTRL+C signal handler to terminate while loop execution

'''
import serial
from signal import signal, SIGINT
import numpy as np 
from time import sleep 

keepRunning = True 

def handler(signal, frame):
    global keepRunning 
    print('SIGINT or CTRL+C detected, setting keepRunning to False')
    keepRunning = False


def serialConnect(portName):
    try: 
        # set read timeout to 1 second
        ser = serial.Serial(portName, baudrate=9600, timeout=1.0)
        print("opened port " + ser.name + '\n')
        
        # give Arduino time to reset
        sleep(3)

        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)


def saveData(filename, numSamples, axData, ayData, azData, gxData, gyData, gzData):

    numElements = numSamples * 6
    data = np.zeros(numElements,dtype=np.int16)
    for i in range(numSamples):
        data[i*6] = axData[i]
        data[i*6+1] = ayData[i]
        data[i*6+2] = azData[i]
        data[i*6+3] = gxData[i]
        data[i*6+4] = gyData[i]
        data[i*6+5] = gzData[i]

    file = open(filename, 'w')
    content = str(data)
    file.write(content)
    file.close()


def main():
        #register the signal handler
    signal(SIGINT, handler)

    portName = "/dev/ttyACM0"
    ser = serialConnect(portName)

    mode = 1        # 1 collect data for calibration
                    # 2 collect raw data with time

 
    totalSamples = 250
    dataCount = 0

    expectedBytes = 12


    # create numpy arrays to store data
    axData = np.zeros(totalSamples,dtype=np.int16)
    ayData = np.zeros(totalSamples,dtype=np.int16)
    azData = np.zeros(totalSamples,dtype=np.int16)
    gxData = np.zeros(totalSamples,dtype=np.int16)
    gyData = np.zeros(totalSamples,dtype=np.int16)
    gzData = np.zeros(totalSamples,dtype=np.int16)
    

    while keepRunning == True and dataCount < totalSamples:
        
        # expecting 12 bytes total
        while ser.in_waiting < expectedBytes:
            pass 

        # read will block until n bytes are read
        bytesRead = ser.read(expectedBytes) 

        if len(bytesRead) < expectedBytes:
           print("len(bytesRead): {} < expectedBytes: {}".format(len(bytesRead),expectedBytes))
           print("Printing integer values of byte array elements")
           for b in bytesRead:
              print(hex(b), end=' ')
           print('\n')
        

        # little endian byte order means the least significant byte contains the most signifcant bits
        # ax, ay, az
        axData[dataCount] = int.from_bytes(bytesRead[0:2], byteorder='little',signed=True)
        ayData[dataCount] = int.from_bytes(bytesRead[2:4], byteorder='little',signed=True)
        azData[dataCount] = int.from_bytes(bytesRead[4:6], byteorder='little',signed=True)

        # gx, gy, gz
        gxData[dataCount] = int.from_bytes(bytesRead[6:8], byteorder='little',signed=True)
        gyData[dataCount] = int.from_bytes(bytesRead[8:10], byteorder='little',signed=True)
        gzData[dataCount] = int.from_bytes(bytesRead[10:12], byteorder='little',signed=True)

        #print("ax: {:d}, ay: {:d}, az: {:d}".format(ax[dataCount], ay[dataCount], az[dataCount])
        #print("gx: {:d}, gy: {:d}, gz: {:d}".format(gxData[dataCount],gyData[dataCount],gzData[dataCount]))
       
        dataCount = dataCount + 1

        if (dataCount % 10) == 0:
            print("dataCount: {}".format(dataCount))
        

    print('while loop terminated')
    ser.close()
    print("closed port")

    print("\naccelerometer data")
    print(axData)
    print(ayData)
    print(azData)

    print("\ngyro data")
    print(gxData)
    print(gyData)
    print(gzData)
    
    saveData('calibrationData.txt', dataCount, axData, ayData, azData, gxData, gyData, gzData)
   



if __name__ == '__main__':
    main()

