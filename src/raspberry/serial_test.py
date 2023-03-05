from arduino_com import ArduinoCommunicator

ard_com = ArduinoCommunicator()
while True:
    x = input("x: ")
    y = input("y: ")
    ard_com.send_angular_error(x, y)
