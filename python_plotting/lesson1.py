import matplotlib.pyplot as plt

#create lists of x and y values
x = [1,2,3,4]
y = [5,7,4,6]

plt.plot(x,y)       # plot x and y using default line style and color

plt.xlabel('x label')
plt.ylabel('y label')
plt.title('Default Arguments\nLine Style & Color')

# everything is drawn in the background, call show to see it
plt.show()