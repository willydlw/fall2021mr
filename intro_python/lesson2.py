'''
lesson 2 - Using string modulus operator % to format output

This method still works in python 3.x and will be familiar to C programmers
'''


# print a with default number of decimal places, f is a placeholder for floating point type
a = 1.234
b = 5
print("a: %f" % (a))

# print a with 1 decimal place
print("a: %.2f" % (a))

# print a as an integer value
print("b: %d" % (b))

# print two variables. The variables to be substituted into the placeholder are placed
# in () separated by commas
msg = 'hello'
print("msg: %s, b: %d" % (msg, b))
