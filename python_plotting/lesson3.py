import matplotlib.pyplot as plt

#create lists of x and y values
x = [1,2,3,4]
y1 = [5,7,4,6]
y2 = [-1, -2, 3, 0]

plt.plot(x,y1, 'r+-')       # color='red', marker='+', linestyle='-'
plt.plot(x, y2, 'bo--')      # color='blue', marker='o', linestyle='--'

plt.xlabel('x')
plt.ylabel('y')
plt.title('Multiple Lines with legend')
plt.legend(['y1', 'y2'])

# everything is drawn in the background, call show to see it
plt.show()