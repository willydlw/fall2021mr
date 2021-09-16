import matplotlib.pyplot as plt
import numpy as np

def lpfilter(x, alpha):
    y = np.zeros(shape=x.shape)
    print('len(y): ', end='')
    print(len(y))
    y[0] = x[0]
    for k in range(1, len(y)):  
        print('k: ' + str(k))
        y[k] = alpha * x[k] + (1-alpha) *  y[k-1]
    return y 



n = 51

t = np.arange(0,51)

#create signals
stepSignal = np.zeros(n)
stepSignal[25:n] = 1
impulseSignal = np.zeros(n)
impulseSignal[25] = 1

#filter step with varying values of alpha
slp1 = lpfilter(stepSignal, 0.1)
slp2 = lpfilter(stepSignal, 0.2)
slp3 = lpfilter(stepSignal, 0.3)
slp6 = lpfilter(stepSignal, 0.6)

# filter impulse with varying values of alpha
ilp1 = lpfilter(impulseSignal, 0.1)
ilp2 = lpfilter(impulseSignal, 0.2)
ilp3 = lpfilter(impulseSignal, 0.3)
ilp6 = lpfilter(impulseSignal, 0.6)


# plot results

plt.figure(1)

plt.plot(t, stepSignal, label='unfiltered')
plt.plot(t,slp1, label="filter, alpha = 0.1")
plt.plot(t,slp2, label="filter, alpha = 0.2")
plt.plot(t,slp3, label="filter, alpha = 0.3")
plt.plot(t,slp6, label="filter, alpha = 0.6")
plt.legend()
plt.grid()
plt.title("Unit Step Response")

plt.figure(2)
plt.plot(t,impulseSignal, label='unfiltered')
plt.plot(t,ilp1, label="filter, alpha = 0.1")
plt.plot(t,ilp2, label="filter, alpha = 0.2")
plt.plot(t,ilp3, label="filter, alpha = 0.3")
plt.plot(t,ilp6, label="filter, alpha = 0.6")
plt.legend()

plt.grid()
plt.title("Impulse Noise Response")

plt.show()