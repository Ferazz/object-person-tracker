import serial
from typing import Dict

class ArduinoCommunicator(serial.Serial):
    """
    Run command: $ sudo adduser your_username dialout
    """
    _CONNECTION_TYPE = {
        "Linux": '/dev/ttyACM0',
        "Windows": "COM4"
    }
    _STD_BAUDRATE = 115200
    
    def __init__(self) -> None:
        super().__init__(self._CONNECTION_TYPE["Windows"], self._STD_BAUDRATE, timeout=1)
        self.reset_input_buffer()
    
    def send_angular_error(self, x, y):
        self.write((f"{x:0>3}\n" + f"{y:0>3}\n").encode('utf-8'))

    def receive_data(self) -> str:
        return self.readline().decode('utf-8').rstrip()

if __name__ == "__main__":
    com = ArduinoCommunicator()
    com.send_angular_error(0, 20)