import serial
from typing import Dict

class ArduinoCommunicator(serial.Serial):
    """
    Run command: $ sudo adduser your_username dialout
    """
    _CONNECTION_TYPE = '/dev/ttyACM0'
    _STD_BAUDRATE = 9600
    
    def __init__(self) -> None:
        super().__init__(self._CONNECTION_TYPE, self._STD_BAUDRATE, timeout=1)
        self.reset_input_buffer()
    
    def send_data(self, data: Dict):
        self.write((data + "\n").encode('utf-8'))

    def recieve_data(self) -> str:
        return self.readline().decode('utf-8').rstrip()
