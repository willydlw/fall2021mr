# Lesson 1 - keyboard input, screen output, data types

A python program, source code, is written in a text editor. The file naming convention is short, all-lowercase names. Underscores may be used if it improves readability. A python program ends with the extension .py

python style guide: https://www.python.org/dev/peps/pep-0008/ 


## lesson1a.py - print string message

A typical first program prints a message. The print function writes to standard output, which is the screen. The simplest usage of the print function is passing a string literal to the function, as shown below. The string literal can be enclosed in single quotes or in double quotes.


```
print('line 1')
print("line 2")
```
</br>

**output**
```
line 1
line 2
```
</br>

Library documentation for print:: https://docs.python.org/3/library/functions.html#print 
</br></br>

## Built-in Data Types

Variables can store data of different types, and different types can do different things.

Python has the following built-in data types. </br>
| Category | Data Type |
| --- | --- |
| Text Type | str |
| Numeric Types | int, float, complex |
| Sequence Types | list, tuple, range |
| Mapping Type |dict |
| Set Types | set, frozenset |
| Boolean Type | bool |
| Binary Types | bytes, bytearray, memoryview |
<br><br>

### Setting the Data Type
In Python, the data type is set when you assign a value to a variable:

| statement | Data Type |
| --- | --- |
| x = 'hello' | str |
| x = 10 | int |
| x = 10.5 | float |
| x = 1j | complex |
| x = ["peach", "orange", "cherry"] | list |
| x = ("peach", "orange", "cherry") | tuple | 	
| x = range(5) | range |
| x = {"name" : "Keisha", "age" : 31} | dict | 	
| x = {"peach", "orange", "cherry"} | set | 	
| x = frozenset({"apple", "banana", "cherry"}) | frozenset | 	
| x = True | bool |
| x = b"Hello" | bytes | 	
| x = bytearray(5) | bytearray | 	
| x = memoryview(bytes(5)) | memoryview | [1](https://www.w3schools.com/python/python_datatypes.asp)
<br><br>

### Getting the Data Type

You can get the data type of any object by using the type() function:

```
x = 10
print(type(x))
```

**output**
```
<class 'int'>
```
</br>

## Variable Names

[PEP8 style guide](https://www.python.org/dev/peps/pep-0008/#function-and-variable-names):  Variable names should be lowercase, with words separated by underscores as necessary to improve readability.</br></br>

## Comments

Comments are used to make a program readable. Comments are not interpreted as source code. The lesson1b source code and other examples contain comments to aid in your understanding. 

**syntax**

Single line comment start with a #. Everything to the right of a # is considered a comment until the end of a line.<br>

```python
# This line is a comment
print('this code will run')    # this is another comment
```

</br>

Multiline comments are wrapped inside triple quotes. Can use single or double quotes.</br>

```python
"""
comment line 1
comment line 2
"""
```

</br>

"Writing Comments in Python(Guide)"  https://realpython.com/python-comments-guide/ </br></br>

## lesson1b.py - printing literals of data types

Run the lesson1b program to see the data types of the various literals. How does print() know how to work with all these different types? Well, the short answer is that it doesn’t. It implicitly calls str() behind the scenes to type cast any object into a string.</br>

```python
print(42)              
print(type(42))                        # <class 'int'>

print(3.14)
print(type(3.14))                      # <class 'float'>

print(1 + 2j)  
print(type(1 + 2j))                    # <class 'complex'>

print(True)
print(type(True))                      # <class 'bool'>

print([1, 2, 3])
print(type([1, 2, 3]))                 # <class 'list'>

print({'red', 'green', 'blue'})
print(type({'red', 'green', 'blue'}))  # <class 'set'>

print({'name':'Baymax', 'age': 3})
print(type({'name':'Baymax', 'age': 3}))  # <class 'dict'>

print('the end')
print(type('the end'))                    # <class 'str'>
```

</br>

## lesson1c.py - keyboard input

The input function reads a line from the keyboard and returns the data as a string type.

The function prototype: input([ *prompt* ])

The brackets mean the prompt argument is optional. The prompt is a character string, written to standard output, without a trailing newline. 

The example below prompts the user to enter their name. It will be followed by a flashing cursor. The function will return when the user presses the enter key.

```python
input('enter your name: ')
```

</br>

**output**
```
enter your name: 
```

</br>

The assignment operator, = , assigns the string returned from the input function to a variable. Note that the data type of name will be dynamically typed to a string, the return data type of the input function.

```python
name = input('enter your name: ')
```

</br>

Run the lesson1c.py source code to see how the input function works. The program prints the data returned by the input function. Note the usage of the + operator in the print functions. When used with strings, the + operator appends (concatenates) strings.

The function type(inMsg) returns the data type of the variable inMsg. It does not return a string type, but is converted to a string by passing it to the str function.

```python
inMsg = input('Enter some text: ')
print("inMsg: " + inMsg)
print("data type of inMsg: " + str(type(inMsg)))
```

</br>


Library documentation for input: https://docs.python.org/3/library/functions.html#input </br></br>

## lesson1d.py - convert user input to integers, floats

The int() and float() constructor functions convert a numeric string argument to a numeric type.

```python
str1 = '10'
intVal = int(str1)

str2 = '10.5'
floatVal = float(str2)
```

</br>

Example lesson1d.py converts user input to integers and floats. Run the program. What happens if the user inputs alphabetic characters or other non-numeric characters?

```python
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
```

<br><br>


## The print() function

The print function can accept any number of positional arguments, including zero, one, or more arguments. 

```python
# print with no arguments produces a blank line
print('line 1')
print()
print('line 3')
```

output
```
line 1

line 3
```

</br>
An example with multiple strings: </br>

```python
# print with 3 arguments
print('today', 'is', 'Wednesday')
```

produces the output shown below. By default, all three string arguments are printed on the same line separated by a space.

```
today is Wednesday
```

</br></br>

To learn more about the print function, you may start an interactive python session and use the help function. Or you may put the following in a program and it will print the same information.

```python
help(print)
```

Generates the following output:

```
Help on built-in function print in module builtins:

print(...)
    print(value, ..., sep=' ', end='\n', file=sys.stdout, flush=False)
    
    Prints the values to a stream, or to sys.stdout by default.
    Optional keyword arguments:
    file:  a file-like object (stream); defaults to the current sys.stdout.
    sep:   string inserted between values, default a space.
    end:   string appended after the last value, default a newline.
    flush: whether to forcibly flush the stream.
```

Apart from accepting a variable number of positional arguments, print() defines four named or keyword arguments, which are optional since they all have default values. 

The separator argument can be a comma, new line, or any type of character.

```python
# comma separator
print('hello', 'world', sep=',')
```

prints the strings, separated by a comma

```
hello,world
```

prints each string separated by a new line

```python
hello
world
```

The keyword end argument appends a newline after printing, by default. Passing the end argument to the print function with a different string changes the behavior.


```python
# terminate first print statement with a space
print('hello', end=' ')
print('world')
```

output
```
hello world
```

```python
# multiple character string arguments
print('List', end='\n  *')
print('bullet point 1', end='\n  *')
print('bullet point 2', end='\n  *')
print('bullet point 3', end='\n\n')
```

output
```
List
  *bullet point 1
  *bullet point 2
  *bullet point 3

```

</br></br>

## String Formatting
</br></br>

### 1. % formatting 

This style was original to python and no longer recommended, as it causes errors with displaying tuples and dictionaries. It is similar to C printf formatting.

Placehoders</br>
%s - string</br>
%d - integer</br>
%f - float</br>

Example usage:

``` python
name = 'Rosey'       # string
age = 27             # integer
pi = 3.14159         # float

print("name: %s, age: %d, favorite number: %f" % (name, age, pi))
```

The values of name, age, pi are inserted into the string placeholders. The default number of decimal places for a float is 6. When inserting more than one variable, you must use a tuple of those variables. 

```
name: Rosey, age: 27, favorite number: 3.141590
```

The number of floating point decimal places can be specified using $.nf where the n is replaced with an integer value. 

Example: show 4 decimal places

```python
pi = 3.14159        
print("favorite number: %.4f" % pi)
```

output 

```
favorite number: 3.1416
``` 

</br></br>

### 2. str.format()

The str.format() placeholders are marked by curly braces. This method was introduced later in python's history. It works with dictionaries and may be easier to use than the % method. However, option 3 is the currently preferred method.

```python
name = 'Rosey'       # string
age = 27             # integer
pi = 3.14159         # float

print("name: {}, age: {}, favorite number: {}".format(name, age, pi))
```

The variables are converted to a string format and printed.

```
name: Rosey, age: 27, favorite number: 3.14159
``` 

</br></br>

### 3. Formatted String Literals

In version 3.6, a new Python string formatting syntax was introduced, called the formatted string literal. These are also informally called f-strings.

An f-string looks very much like a typical Python string except that it’s prepended by the character f. You can use single, double, or triple quotes, just like any other string.

```python
msg = f'hello world'
print(msg)
print(f"hello mars")
print(f'''hello venus''')
print(f"""hello sun""")
```

output
```
hello world
hello mars
hello venus
hello sun
```

</br>
Formatted string literals are string literals that have an f at the beginning and curly braces containing expressions that will be replaced with their values. The expressions are evaluated at runtime.</br></br>

```python
name = 'Rosey'       # string
age = 27             # integer
pi = 3.14159         # float

print(f"name: {name}, age: {age}, favorite number: {pi}")
```

output
```
name: Rosey, age: 27, favorite number: 3.14159
```

Because f-strings are evaluated at runtime, you can put any and all valid Python expressions in them. The example below calls the string function to convert to lower case and perfoms a math operation in the placeholder.

```python
name = 'Rosey'       # string
age = 27             # integer

print(f"{name.lower()}")
print(f"twice my age is {2*age}")
```

output
```
name = 'Rosey'       # string
age = 27             # integer

print(f"{name.lower()}")
print(f"twice my age is {2*age}")
```

For more information on formatted string literals, https://docs.python.org/3/reference/lexical_analysis.html#f-strings 