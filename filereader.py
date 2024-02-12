import numpy as np
from collections import Counter



with open('Assignment1/datafile.txt','r') as f: #opens file
     b = eval(f.read())
array=np.array(b) #makes a numpy type from the lst
b_max=max(array)#finds the max
b_min=min(array)#finds the min
b_index_array=np.where(array==38)#finds where 38 is 
b_index=max(b_index_array)#finds the max
most_common_array=np.bincount(array)#makes a binary count array
most_common_max=max(most_common_array)#finds the occurenes of the most common number
most_common_value_array=np.where(most_common_array==most_common_max)#finds the actual number that is most common
most_common_value=max(most_common_value_array)

sorted_array=np.sort(array)#sorts array

print(f"The max value is {b_max} \n")
print(f"The min value is {b_min} \n")
print(f"The location of 38 is {b_index} \n")
print(f"The most common value is {most_common_value} that occurs {most_common_max} times! \n")
print(f"Here is my sorted array {sorted_array}")

