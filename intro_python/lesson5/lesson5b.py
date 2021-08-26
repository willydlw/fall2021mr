# define function with one parameter
def my_function(msg):
   print('my_function, msg: ', msg)

# call function my_function, pass 'cat in the hat' as an argument
# to function parameter msg
my_function('cat in the hat')


# define function with multiple parameters
def print_names(fname, lname):
   print(fname + " " + lname)

# call function print_names. Pass two arguments to its parameters
# When calling the function, it expects two arguments because the 
# function was defined with two parameters. Try calling it with one or
# three parameters. You will get an error message.
print_names('Bart', 'Simpson')