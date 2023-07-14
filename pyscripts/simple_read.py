#import os
import csv
#print(os.getcwd())

#new_directory = "C:\\Users\\Nikos\\Documents\\Arduino\\libraries\\ActiveStepperActuator\\pyscripts"
#os.chdir(new_directory)


filename = "test"

with open(filename, "r") as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        print(row)      