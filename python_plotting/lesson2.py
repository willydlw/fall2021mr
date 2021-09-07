import matplotlib.pyplot as plt

#create lists of x and y values
x = [1,2,3,4]
y = [5,7,4,6]

# keyword arguments
plt.plot(x, y,color='green', marker='o', linestyle='dashed', \
            linewidth=2, markersize=12)

plt.xlabel('x label')
plt.ylabel('y label')
plt.title('Keyword Arguments\nLine Style & Color')

# everything is drawn in the background, call show to see it
plt.show()