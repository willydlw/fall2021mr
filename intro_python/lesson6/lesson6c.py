# lesson6c - arbitrary number of arguments

# arbitrary arguments, *args 
def some_function(*names):
   print('there are ' + str(len(names)) + ' names')
   for n in names:
      print(n)


# call some_function.
some_function('Bart', 'Homer', 'Marge', 'Lisa')
