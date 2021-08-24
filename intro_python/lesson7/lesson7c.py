# lesson 7c- append to list

# create an empty list
myList = []

# print the number of items in the list and the list contents
print("empty list")
print("len(myList): ", len(myList))
print("myList: ", myList)

dataRead = 1

while dataRead != 'q':
   dataRead = input("\nenter list item or q to quit: ")
   myList.append(dataRead)
   print("len(myList): ", len(myList))
   print("myList: ", myList)
