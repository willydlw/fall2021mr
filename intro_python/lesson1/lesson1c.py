# lesson1c - converts strings to integer, float 

# convert the string to an integer
# if you type a non-numeric value, the conversion to int will fail
integerValue1 = int(input("\nenter integer value: "))

# when using the print function, the + operator appends to
# a string. The integer is converted to a string for printing
print("integerValue1: " + str(integerValue1))
print("data type of integerValue1: " + str(type(integerValue1)))

#convert the string to a float
floatValue1 = float(input("\nenter float value: "))

# when using the print function, the + operator appends to
# a string. The float is converted to a string for printing
print("integerValue1: " + str(floatValue1))
print("data type of floatValue1: " + str(type(floatValue1)))
