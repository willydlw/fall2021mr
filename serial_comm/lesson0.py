'''
    Prints list of connected serial com ports
'''
import serial
import serial.tools.list_ports 


# list all connected serial com ports
ports = list(serial.tools.list_ports.comports())

print("\nPrinting connected serial port list\n")

for p in ports:
    print("\ndevice: " , p.device)
    print("device path: " , p.device_path)
    print("description: " , p.description)
    print("manufactuer: " , p.manufacturer)
    print("serial number: " , p.serial_number)
