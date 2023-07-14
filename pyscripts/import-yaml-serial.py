import serial
import yaml
import os
print(os.getcwd())
abs_path = os.path.abspath("yaml.txt")
print(abs_path)

file_directory = os.path.dirname(abs_path)
os.chdir(file_directory)

# Open the serial connection
arduinoData = serial.Serial('COM8', 115200)

# Read the data from the text file
with open("C:\\Users\\Nikos\\Documents\\Arduino\\libraries\\ActiveStepperActuator\\yaml.txt","r") as file:
    data = yaml.safe_load(file)

# Wait for the command from the Arduino
while True:
    if arduinoData.inWaiting() > 0:
        command = arduinoData.readline().decode().strip()
        if command == 'INIT_DATA':
            # Receive the list of variable names from the Arduino
            variables_request = arduinoData.readline().decode().strip()
            variable_names = variables_request.split(',')

            # Prepare the response
            response = []

            # Find the corresponding values for the requested variables
            for variable_name in variable_names:
                if variable_name in data:
                    variable_value = data[variable_name]
                    response.append(variable_value)
                else:
                    response.append(None)  # Variable not found

            # Send the response back to the Arduino
            response_str = ','.join(str(value) for value in response)
            arduinoData.write(response_str.encode())
            break

# Close the serial connection
arduinoData.close()