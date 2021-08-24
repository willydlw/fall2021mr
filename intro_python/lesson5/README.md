# Lesson 5 - loops

Programming statements are generally executed sequentially. The first statement in a function is executed first, followed by the second, and so on. There are situations when a block of code needs to be executed more than once. A loop statement allows us to execute a statement or group of statements multiple times. The following diagram illustrates a loop statement.
<br><br>

![loop structure](./images/loop_architecture.jpg) [1](https://www.tutorialspoint.com/python3/images/loop_architecture.jpg)

<br><br>

The python language two types of loops and allows nesting of loops.
- while loop
- for loop
<br><br>

## while loop

Repeats a statement or group of statements while a given condition is TRUE. It tests the condition before executing the loop body.

![while loop](./images/python_while_loop.jpg) [2](https://www.tutorialspoint.com/python3/images/python_while_loop.jpg)
<br><br>

**syntax**
```
while expression:
   statement(s)
```
<br>

Note: statement(s) inside the body of the loop must be indented with a tab.<br><br>

### lesson5a.py

```
count = 0

print("before loop")

while (count < 3):
   print("while loop, count: ", count)
   count = count + 1

print("after loop")
```
<br>

**output**

```
before loop
while loop, count:  0
while loop, count:  1
while loop, count:  2
after loop
```
<br><br>

## for loop

The for statement in Python has the ability to iterate over the items of any sequence, such as a list or a string. <br>
<br>

![for loop](./images/python_for_loop.jpg) [3](https://www.tutorialspoint.com/python3/images/python_for_loop.jpg)
<br><br>


**syntax**

```
for iterating_var in sequence:
   statements(s)
```
<br>

If a sequence contains an expression list, it is evaluated first. Then, the first item in the sequence is assigned to the iterating variable iterating_var. Next, the statements block is executed. Each item in the list is assigned to iterating_var, and the statement(s) block is executed until the entire sequence is exhausted.

Python has several data structures that may contain a sequence of items. The next tutorial, lesson 6, discusses the list data structure.<br><br>

### lesson5b.py 

In the example code below, we create a list named fruits and then use a for loop to access and print each item in the list. The for loop executes if the sequence is not empty. The iterating_var, f, is assigned the first item in the list: 'watermelon', the print function then prints 'watermelon'. The loop continues iterating through the entire sequence, until it reaches the end.

```
# create a list of string data types
fruits = ['watermelon', 'grapes', 'apples']

print("iterate over the fruits list")
# use a for loop to iterate through the list
for f in fruits:
   print(f)
```
<br>

**output**
```iterate over the fruits list
watermelon
grapes
apples
``` 
<br>
This works for other data types as well. The example code below creates a list of integers and uses a for loop to iterate through the list.<br>

```
# create a list of integers
values = [ 17, -3, 99, 0]
for v in values:
   print("v: {}, type(v): {}".format(v, type(v)))
```
<br>

**output**
```
v: 17, type(v): <class 'int'>
v: -3, type(v): <class 'int'>
v: 99, type(v): <class 'int'>
v: 0, type(v): <class 'int'>
```
<br><br>

## lesson5c.py - range function

range(start, stop[, step])

range takes three arguments. Out of the three, two arguments are optional. I.e.,start and step are the optional arguments.

- A start argument is a starting number of the sequence. i.e., lower limit. By default, it starts with 0 if not specified.
- A stop argument is an upper limit. i.e., generate numbers up to this number. The range() doesn’t include this number in the result. The step is a difference between each number in the result. The default value of the step is 1 if not specified.<br>

**syntax**
```
for var in range():
```
<br>

The range function can be used to iterate through the list of fruits, as shown below. When start and step are not defined, it will start with the first item in the list and stop after the list is exhausted. The list item is accessed using [index] notation.<br>

```
# create a list of string data types
fruits = ['watermelon', 'grapes', 'apples']

# use the python len() function to return number of objects in the list
print("\nfor loop range(len(fruits)) output")
for index in range(len(fruits)):
   print('Current fruit : ', fruits[index])
```
<br>

**output**
```
for loop range(len(fruits)) output
Current fruit :  watermelon
Current fruit :  grapes
Current fruit :  apples
```
<br><br>


The example below illustrates using the range function with a stopping integer value. By default it will start with 0, increment by 1, and stop loop execution when reaching 4.

```
# python range function: range(start, stop, step)
# i starts at 0, increases by 1, and stops after 3
print('\nfor loop range(4)')
for i in range(4):
   # end print statement with a comma instead of a newline
   print(i, end=', ')
```
<br>

**output**
```
for loop range(4)
0, 1, 2, 3, 
```
<br><br>

The final example illustrates using start, stop, and step arguments.

```
print("\n\nfor loop range(3,8,2)")
# i starts at 3, stops after 7, increments by 2
for i in range(3, 8, 2):
   print(i, end=' ')

print('')
```
<br>

**output**
```
for loop range(3,8,2)
3 5 7 
```
<br>


<br><br>
