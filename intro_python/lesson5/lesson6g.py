# lesson6g - returning data from a function

import math       # sin, cos, pi 

# return one value
def scale_it(x):
   return 10 * x 

scaledVal = scale_it(3)
print("scaledVal: ", scaledVal)

# return two values
def find_xy(radius, angle):
   x = radius * math.cos(angle)
   y = radius * math.sin(angle)
   return x, y 

# assigns x to a, b to y
a, b = find_xy(9, math.pi)
print("a: {}, b: {}".format(a,b))