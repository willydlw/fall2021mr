# lesson6d - keyword arguments
def average_of_three(a, b, c):
   avg = (a+b+c)/3
   print('average of a: {}, b: {}, c: {} is {}'.format(a,b,c,avg))


# call average_of_three, use key=value syntax
# does not require that we pass arguments in same order as parameters
average_of_three(c = 9, a = 7, b = 4)

