# Introduction to Python Programming

"Python is an easy to learn, powerful programming language. It has efficient high-level data structures and a simple but effective approach to object-oriented programming. Python’s elegant syntax and dynamic typing, together with its interpreted nature, make it an ideal language for scripting and rapid application development in many areas on most platforms."[1](https://docs.python.org/3/tutorial/index.html)


The are many good python programming tutorials found on the Internet, as well as many good books to help you get started with python. In this module, we will cover just a few python basics needed to get started in this course.


## Books and Tutorial References

One of my favorite books: Python Crash Course, 2nd Edition: A Hands-On, Project-Based Introduction to Programming by Eric Mathes,  https://www.amazon.com/Python-Crash-Course-2nd-Edition/dp/1593279280



An internet search with keywords "python tutorial for beginners" will provide a myriad of choices. Tutorials point has good tutorials for many languages.

https://www.tutorialspoint.com/python/


There are many of YouTube channels providing tutorials. You may find one of the following channels helpful.

- Microsoft's Python for Beginners is a good series of short videos, including helping you configure Visual Studio Code. https://channel9.msdn.com/Series/Intro-to-Python-Development?WT.mc_id=python-c9-niner 

- techwithtim https://www.youtube.com/playlist?list=PLzMcBGfZo4-mFu00qxl0a67RhjjZj3jXm
- sentdex https://www.youtube.com/playlist?list=PLQVvvaa0QuDeAams7fkdcwOGBpGdHpXln
- corey shafer https://www.youtube.com/playlist?list=PL-osiE80TeTt2d9bfVyTiXJA-UTHn6WwU


The python language documentation also contains a set of tutorials and library documentation:

- The Python Tutorial https://docs.python.org/3/tutorial/index.html 
- The Python Standard Library  https://docs.python.org/3/library/index.html#library-index 



You will find there are references to python 2.x and 3.x. There are differences between these versions. Class examples will be written for python 3.x.




### How does python work?

Python is an interpreted, high-level, general-purpose programming language. It is dynamically typed and garbage-collected.

Python code, written in .py file is first compiled to *bytecode*, a low-level set of instructions that can be executed by an interpreter. Instead of executing the instructions on the CPU, bytecode instructions are executed on a Virtual Machine.

Dynamic typing means that the interpeter determines the validity of variable types and operations performed at run-time.

The Garbage Collector automatically frees up memory space. It keeps track of the number of references to an object. When the reference count goes down to zero, it deletes the object.


The example in lessons 1 - 7 below are intended to introduce you to the python syntax and provide a quick start reference for the language.

<br><br>

## Lesson 1 - keyboard input, print output, converting strings to int, float

Lesson 1 demonstrates how to read input from the keyboard, convert the data from string to integer or float, and print the data. See lesson1.py.



## Lesson 2 - Formatting output using String modulo operator(%) 

The % operator can also be used for string formatting. It interprets the left argument much like a printf()-style format string to be applied to the right argument. In Python, there is no printf() function but the functionality of the ancient printf is contained in Python. To this purpose, the modulo operator % is overloaded by the string class to perform string formatting. Therefore, it is often called string modulo (or sometimes even called modulus) operator.

```
# print a with default number of decimal places, f is a placeholder for floating point type
a = 1.234
b = 5
print("a: %f" % (a))

# print a with 1 decimal place
print("a: %.2f" % (a))

# print b an integer value
print("b: %d" % (b))

# print two variables. The variables to be substituted into the placeholder are placed
# in () separated by commas
msg = 'hello'
print("msg: %s, b: %d" % (msg, b))
```

The newer method of formatting uses .format(), where {} become the placeholders, and the arguments are passed to the format function. See example below. https://docs.python.org/3/library/string.html


```
a = 1.234
b = 5
print("a: {}".format(a))
print("a: {}, b: {}".format(a,b))
```
<br><br>

## Lesson 3 - Arithmetic Operators

| Operator | Description | Example a = 10, b = 20 |
| --- | --- | --- |
| + | Addition 	Adds values on either side of the operator. | a + b = 30 |
| - |Subtraction 	Subtracts right hand operand from left hand operand. | a – b = -10 |
| * | Multiplication 	Multiplies values on either side of the operator | a * b = 200 |
| / | Division 	Divides left hand operand by right hand operand | b / a = 2 |
| % | Modulus 	Divides left hand operand by right hand operand and returns remainder | b % a = 0 |
| ** | Exponent 	Performs exponential (power) calculation on operators | a**b = 10 to the power 20 |


https://www.tutorialspoint.com/python/python_basic_operators.htm

See lesson3.py for example usage
<br><br>

## Lesson 4 - Comparison Operators, if statements

| Operator | Description | Example a = 10, b = 20 |
| --- | --- | --- |
| == | If the values of two operands are equal, then the condition becomes true. | (a == b) is not true. |
| != | If values of two operands are not equal, then condition becomes true. | (a != b) is true. |
| <> | If values of two operands are not equal, then condition becomes true. | (a <> b) is true. This is similar to != operator. |
| >  | If the value of left operand is greater than the value of right operand, then condition becomes true. | (a > b) is not true. |
| <  | If the value of left operand is less than the value of right operand, then condition becomes true. | (a < b) is true. |
| >= | If the value of left operand is greater than or equal to the value of right operand, then condition becomes true. | (a >= b) is not true. |
| <= | If the value of left operand is less than or equal to the value of right operand, then condition becomes true. | (a <= b) is true. |

<br>

**if statements**
- An if statement consists of a boolean expression followed by one or more statements.
- An if statement can be followed by an optional else statement, which executes when the boolean expression is FALSE.
- nested if statements
- - You can use one if or else if statement inside another if or else if statement(s).

In Python, all the statements indented by the same number of character spaces after a programming construct are considered to be part of a single block of code. Python uses indentation as its method of grouping statements.

See lesson4.py for example usage <br>
<br><br>


## Lesson 5 - loops

Lesson 5 directory covers while loops, for loops, and range function.

## Lesson 6 - functions

Lesson 6 provides multiple examples of function syntax.

## Lesson 7 - lists

Lesson 7 illustrates list creation, appending to end of a list, and sorting a list.