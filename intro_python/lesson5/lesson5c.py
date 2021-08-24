# lesson 5c - for loop using range function

# create a list of string data types
fruits = ['watermelon', 'grapes', 'apples']

# use the python len() function to return number of objects in the list
print("\nfor loop range(len(fruits)) output")
for index in range(len(fruits)):
   print('Current fruit : ', fruits[index])



# python range function: range(start, stop, step)
# i starts at 0, increases by 1, and stops after 3
print('\nfor loop range(4)')
for i in range(4):
   # end print statement with a comma instead of a newline
   print(i, end=', ')

print("\n\nfor loop range(3,8,2)")
# i starts at 3, stops after 7, increments by 2
for i in range(3, 8, 2):
   print(i, end=' ')

print('')