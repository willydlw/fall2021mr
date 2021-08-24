# lesson 6e - arbitrary keyword arguments

# aribtrary keyword arguments, **kwargs
def print_knames(**person):
   print("Last name is " + person["lname"])

# call function, specify keyword arugments
print_knames(fname='Charlie', lname = 'Brown')
