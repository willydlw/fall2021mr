# lesson 7a - getting started with lists

# create a list that contains string data types
school_supplies = ['pencil', 'paper', 'glue', 'calculator', 'notebook']

# what is the school_supplies data type?
print("type(school_supplies) ", type(school_supplies))

# print all items in the list
print(school_supplies)


# access list items by index number
print("\naccess list item at index 2")
print(school_supplies[2])
school_supplies[2] = 'paste'
print(school_supplies[2])


# iterate through the list, access each object individually
print("\niterating through list with for loop")
for s in school_supplies:
   print(s)