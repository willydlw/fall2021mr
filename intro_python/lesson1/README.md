# Lesson 1 - keyboard input, screen output, data types

A python program, source code, is written in a text editor. The file naming convention is short, all-lowercase names. Underscores may be used if it improves readability. A python program ends with the extension .py

python style guide: https://www.python.org/dev/peps/pep-0008/ 


## lesson1a.py - print string message

A typical first program prints a message to the screen. The print function writes to the screen, also known as standard output. The simplest usage of the print function is passing a character string to the function, as shown below. The character string can be enclosed in single quotes or in double quotes.


```
print('line 1')
print("line 2")
```
<br>

**output**
```
line 1
line 2
```
<br>

Library documentation for print:: https://docs.python.org/3/library/functions.html#print 
<br><br>

## lesson1b.py - keyboard input

The input function reads a line from the keyboard and returns the data as a string type.

The function prototype: input([ *prompt* ])

The brackets mean the prompt argument is optional. The prompt is a character string, written to standard output, without a trailing newline. 

The example below prompts the user to enter their name. It will be followed by a flashing cursor. The function will return when the user presses the enter key.

```
input('enter your name')
```
<br>

**output**
```
enter your name: 
```
<br>

The assignment operator, = , assigns the string returned from the input function to a variable. Note that the data type of name will be dynamically typed to a string, the return data type of the input function.

```
name = input('enter your name')
```
<br><br> 

### Comments

Comments are used to make a program readable. Comments are not interpreted as source code. The lesson1b source code and other examples contain comments to aid in your understanding. 

**syntax**

Single line comment start with a #. Everything to the right of a # is considered a comment until the end of a line.<br>

```
# This is a comment
print('this code will run')    # this will not run, is a comment
``` 
<br>

Multiline comments are wrapped inside triple quotes. Can use single or double quotes. 

```
"""
comment line 1
comment line 2
"""
```
<br>

"Writing Comments in Python(Guide)"  https://realpython.com/python-comments-guide/ 

<br>


Run the lesson1b.py source code to see how the input function works. The program prints the data returned by the input function. Note the usage of the + operator in the print functions. When used with strings, the + operator appends (concatenates) strings.

The function type(inMsg) returns the data type of the variable inMsg. It does not return a string type, but is converted to a string by passing it to the str function.

```
inMsg = input('Enter some text: ')
print("inMsg: " + inMsg)
print("data type of inMsg: " + str(type(inMsg)))
```
<br>


Library documentation for input: https://docs.python.org/3/library/functions.html#input

<br><br>

## lesson1c.py - convert user input to integers, floats

The int() and float() constructor functions convert a numeric string argument to a numeric type.

```
str1 = '10'
intVal = int(str1)

str2 = '10.5'
floatVal = float(str2)
```

### Built-in Data Types

Variables can store data of different types, and different types can do different things.

Python has the following data types built-in by default, in these categories:
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
<br>

Example lesson1c.py converts user input to integers and floats. Run the program. What happens if the user inputs alphabetic characters or other non-numeric characters?

```
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