# lesson 7b 
#   create a list of integers
#   sort the list

myIntList = [6, 3, 7, -2, -13, 0]
print("original list: ", myIntList)

# iterator through the list one item at a time
# print the list item and its data type
print("\ndata type of each list item")
for v in myIntList:
   print("v: ", v, type(v))


# sort the list
myIntList.sort()
print("\nsorted list: ", myIntList)