# Lesson 6 - functions

A function is a block of code that only runs when it is called. Data can be passed to a function's parameters. A function can also return data.
<br><br>


## lesson6a.py - define a function with no parameters, call the function

A function is defined using the **def** keyword, followed by the function name, parentheses, optional parameters inside the parentheses and a colon. 

The following example defines a function that prints a message when called. It has no arguments and does not return any data. Statements inside the function body are indented with a tab. Note that the function is defined in the program before it is called. Think of the python interpreter as starting at the top of the file and reading through all the code in the file. When it reads the function definition, it registers its existence, but knows not to execute the code until the function is called. The programming statement `hello_function()` is the function call that causes control flow to execute the code contained in the function.

```
# define function with no parameters
def hello_function():
   # function may contain multiple statements
   # this simple example uses only a print statement
   print('hello from hello_function')

# code begins execution here
# call function hello_function
hello_function()
```
<br>

**output**

```
hello from hello_function
```
<br><br>

## lesson6b.py - functions with parameters

The example function has one parameter in the list. The parameter name is msg. When the function is called, the argument data in the function call will be passed to the function parameter. The function does not return any data.<br>

```
# define function with one parameter
def my_function(msg):
   print('my_function, msg: ', msg)

# call function my_function, pass 'cat in the hat' as an argument
# to function parameter msg
my_function('cat in the hat')
```

**output**
```
my_function, msg:  cat in the hat
```
<br>
The next example function definition has two parameters. Notice the parameter list is comma-separated. When calling the function, the program must pass two arguments. There is a one-to-one correspondence in the order of assigments. The first argument in the function call is assigned to the first parameter. The second argument in the function call is assigned to the second parameter. 

```
def print_names(fname, lname):
   print(fname + " " + lname)

# call function print_names. Pass two arguments to its parameters
# When calling the function, it expects two arguments because the 
# function was defined with two parameters. Try calling it with one or
# three parameters. You will get an error message.
print_names('Bart', 'Simpson')
```
<br>

**output**
```
Bart Simpson
```

<br><br>

## lesson6c.py -  Arbitrary Arguments, *args

If you do not know how many arguments will be passed into a function, add a * before the parameter name in the function definition. The function will receive a *tuple* of arguments and can access the items accordingly. (A tuple is a collection of objects which ordered and immutable.)

Note this is an advanced concept which you will likely not code in the class, but will see *args in python library function definitions. The main takeaway is to understand that *args means you can pass an arbitrary number of arguments to the function.
<br>

```
# arbitrary arguments, *args
def some_function(*names):
   print('there are ' + str(len(names)) + ' names')
   for n in names:
      print(n)

# call some_function.
some_function('Bart', 'Homer', 'Marge', 'Lisa')
```
<br>

**output**
```
there are 4 names
Bart
Homer
Marge
Lisa
```
<br><br>

## lesson6d.py - Keyword Arguments, key=value

Arguments do not have to be in the same order as the parameters when using key = value syntax. Below, the value passed to c is defined in the function call as c = 9. Similarly, the values for b and a are defined with key=value syntax.

```
# keyword arguments
def average_of_three(a, b, c):
   avg = (a+b+c)/3
   print('average of a: {}, b: {}, c: {} is {}'.format(a,b,c,avg))

# call average_of_three, use key=value syntax
# does not require that we pass arguments in same order as parameters
average_of_three(c = 9, a = 7, b = 4)
```
<br>

**output**

```
average of a: 7, b: 4, c: 9 is 6.666666666666667
```
<br><br>

## lesson6e.py -  Arbitrary Keyword Arguments, **kwargs

If you do not know how many keyword arguments will be passed into your function, add two asterisk: ** before the parameter name in the function definition.

The function will receive a dictionary of arguments, and can access the items accordingly.

```
# aribtrary keyword arguments, **kwargs
def print_knames(**person):
   print("Last name is " + person["lname"])

# call function, specify keyword arugments
print_knames(fname='Charlie', lname = 'Brown')
```
<br>

**output**
```
Last name is Brown
```
<br><br>

## lesson6f.py -  Default Parameter Value

Default values may be assigned to a parameter, when the function is called without an argument.

```
# default parameter value
def my_university(school = "CU Denver"):
   print("My university is " + school)

# call function, use default value
my_university()

# call function, pass value to parameter
my_university("Montana State")

```
<br>

**output**
```
My university is CU Denver
My university is Montana State
```
<br><br>

## lesson6g.py -  Return Values

Use the return statement to return data. python allows returning multiple objects from a function. When returning more than one object, comma separate the list. The example below imports the math library to access the sin, cos functions and the constant pi.

```
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
```
<br>

**output** <br>
Note: We expect b's value to be zero as the sin(pi) is 0. Due to floating point precision limitations, we see a value very close to zero, but not exaclty zero. <br>
```
scaledVal:  30
a: -9.0, b: 1.102182119232618e-15
```
<br><br>

## lesson6h.py -  pass statement

Programmers sometimes define a function before writing the function code. Writing a function with no statements in the body causes an error. The pass statement is used to avoid an error. It allows the function to be called.

```
# pass statement allows function to be called
def emptyFunction():
   pass 

# call emptyFunction
emptyFunction()
```
