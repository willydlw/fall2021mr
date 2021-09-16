import matplotlib.pyplot as plt
import numpy as np


def lpfilter(x, alpha, x0):
    y = np.zeros(shape=x.shape)
    yp = x0
    print(yp)
    for k in range(len(y)):  
        yp += alpha * (x[k]-yp)
        y[k] = yp
    return y 



np.random.seed(123456789)   # repeatable results

# ideal signal frequency: cycles per second, Hz
f0 = 4

# Sampling
numSamples = 65536
simTime = 1.0                   # seconds
fs = numSamples /  simTime      # sampling frequency, samples/sec


# create an array of sample time points, evenly spaced
t = np.arange(0,simTime,simTime/numSamples)

# create square wave signal, peak-to-peak amplitude of 2.0
idealSignal = (np.mod(f0*t,1) < 0.5)*2.0-1

# add random noise from standard normal distribution, mean 0, variance 1
noiseSignal = 0.5*np.random.randn(*idealSignal.shape)

# filter signal
deltaT = t[1] - t[0]        # time difference between samples
fc = 1000                    # cutoff frequency
tau = 1/fc                     
alpha = deltaT / tau 

print('tau: ' + str(tau))
print('deltaT: ' + str(deltaT))

lpFiltered = lpfilter(idealSignal+noiseSignal, alpha, 0)


plt.figure(1, figsize=(8,6))
plt.plot(t, idealSignal+noiseSignal, 'gray', label='noise')
plt.plot(t, idealSignal, 'green', linewidth=3, label='ideal')
plt.title('Unfiltered Noisy Signal')
plt.xlabel('time [sec]')
plt.legend()
plt.grid()

plt.figure(2, figsize=(8,6))
plt.plot(t, lpFiltered, label='alpha: ' + str(alpha))
plt.legend()
plt.title('Filtered Signal\ncutoff frequency: '+str(fc))
plt.xlabel('time [sec]')

# frequency spectrum
idealSpectrum = np.abs(np.fft.ifft(idealSignal))
noiseSpectrum = np.abs(np.fft.ifft(noiseSignal))

# frequency plotting variables
endFreq = 1000                      # plot 0 Hz to endFreq
f = np.arange(endFreq)              # array of frequency values 

# plot the energy, separate the ideal and noisy spectrum
plt.figure(3, figsize=(8,6))
plt.subplot(2,1,1)
plt.plot(f, idealSpectrum[:endFreq], color='green',label='ideal')
plt.ylabel('amplitude')
plt.xlim(0,endFreq)
plt.title('Frequency Spectrum')
plt.legend()

plt.subplot(2,1,2)
plt.plot(noiseSpectrum[:endFreq], color='yellow', label='noise')
plt.xlabel('frequency, Hz')
plt.ylabel('amplitude')
plt.xlim(0,endFreq)
plt.legend()


# create log scale plots

for N in [1000, 8000]:
    f = np.arange(N)
    plt.figure(figsize=(8,6))
    # log plot does not like 0, add small near-zero value to avoid program crash
    plt.semilogy(f,1e-10+idealSpectrum[:N],color='green')
    plt.semilogy(f,1e-10+noiseSpectrum[:N], color='yellow')
    plt.xlim(0,N)
    plt.ylim(1e-4,1)
    plt.legend(('signal','noise'))
    plt.xlabel('frequency, Hz')
    plt.ylabel('amplitude')
    plt.title('Spectrum Log Plot')

plt.show()
