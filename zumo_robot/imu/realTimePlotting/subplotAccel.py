'''
Description: Plot accelerometer data
    Serial reads accelerometer x,y,z 
    Plots data in subplots
        Removes oldest data from array
        Appends new data
'''

import serial
import numpy as np
from time import sleep 
from matplotlib import pyplot as plt
from matplotlib import animation


def serialConnect(portName, baudRate):
    try: 
        ser = serial.Serial(portName, baudRate)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser 
    except serial.SerialException:
        raise IOError("problem connecting to " + portName)


def init():
    graphX.set_data([], [])
    graphY.set_data([], [])
    graphZ.set_data([], [])
    return graphX, graphY, graphZ

def animate(i):
    global t, accelx, accely, accelz

    while (ser.inWaiting() == 0):
        pass

    arduinoString = ser.readline().decode("utf-8")
    # remove \r\n from end of string, then use comma delimiter to split
    dataArray = arduinoString.strip().split(',')

    # add scaled data points to end of list
    # 32767/2 scales to units of g
    accelx.append(float(dataArray[0])/(32767/2))    
    accely.append(float(dataArray[1])/(32767/2))    
    accelz.append(float(dataArray[2])/(32767/2))

    # remove oldest data point from front of list
    accelx.pop(0)
    accely.pop(0)
    accelz.pop(0)

    # add data to each graph set for plotting
    graphX.set_data(t, accelx)
    graphY.set_data(t, accely)
    graphZ.set_data(t, accelz)

    # return list of iterable artists
    # animate documentation says we must return these
    return graphX, graphY, graphZ

   

if __name__ == '__main__':

    portName = "/dev/ttyACM0"
    ser = serialConnect(portName,9600)
    sleep(2)                                        # give Arduino time to reset

    # flush input buffer, discarding all contents
    ser.reset_input_buffer()

    numPoints = 201                                 # number of data points
    fig = plt.figure(figsize=(12, 6))               # create figure window

    # add sub plots 
    ax1 = fig.add_subplot(3, 1, 1)
    ax2 = fig.add_subplot(3, 1, 2)
    ax3 = fig.add_subplot(3, 1, 3)

    # get Line2D object for each sub plot
    graphX, = ax1.plot([], [], 'b', label = 'X')
    graphY, = ax2.plot([], [], 'r', label = 'Y')
    graphZ, = ax3.plot([], [], 'g', label = 'Z')
    axes = [ax1, ax2, ax3]

    # specify axes limits, labels, and legend location
    for ax in axes:
        ax.set_xlim(0, numPoints-1)
        ax.set_ylim(-2, 2)
        ax.set_ylabel('Acceleration [G]')
        ax.legend(loc='upper right')
        ax.grid(True)

    ax1.set_title('Real-time sensor data')
    ax3.set_xlabel('Data points')
    
    
    t = list(range(0, numPoints))
    accelx = []
    accely = []
    accelz = []

    for i in range(0, numPoints):
        accelx.append(0)
        accely.append(0)
        accelz.append(0)


    delay = 20
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                               interval=delay, blit=True)

    plt.show() 

