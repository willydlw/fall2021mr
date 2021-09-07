'''
Objective: Illustrate how sample frequency affects 
representation of time domain signal. 

'''
import math
import matplotlib.pyplot as plt 

# signal parameters
signalFrequency = 1                    # units: Hz, cycles/sec
signalPeriod   = 1 / signalFrequency   # units: sec/cycle

# sample parameters
sampleFrequency = signalFrequency * 16  # units: samples/sec
samplePeriod = 1 / sampleFrequency     # units: sec/sample  (time between samples)


numCycles = 1                                  # number of signal cycles
totalSamples = int(numCycles/samplePeriod)     # units: cycles/(sec/samplematp)
n = 0                                  # sample number


sinValues = []                   # create empty list
timeValues = []

while(n < totalSamples):
   sinValues.append(math.sin(2*math.pi*signalFrequency*n*samplePeriod))
   timeValues.append(n*samplePeriod)
   n = n + 1

print(sinValues)

plt.plot(timeValues, sinValues, 'b*')
plt.title('Sine Wave, ' + str(signalFrequency) + ' Hz')
plt.xlabel('seconds')
plt.ylabel('amplitude')
plt.show()
