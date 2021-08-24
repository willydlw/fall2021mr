# lesson 4: if else if else  logic example


x = int(input('enter an integer: '))

# if (comparison)
if x < 0:
   # use tab to indent code to be executed when conditional expression is true
   # may have one or more statements
   print('negative')
   z = x * 2

# elif stands for else if
elif x == 0:
   print('zero')

# else executes when every test has been false
else:
   print('positive')
