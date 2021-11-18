from serial import Serial


class ArduinoPy:
    def __init__(self, port="COM4", baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.arduino = Serial(
            port=self.port, baudrate=self.baudrate, timeout=self.timeout
        )

    def read_control_data_from_bert(self):
        control_data = self.arduino.readline()
        print(control_data)
        if self.control_data_verified(control_data):
            return control_data
        raise Exception("Incorrect Control Data Format")

    def control_data_verified(self, control_data):
        """Verify control data is in the correct format"""

        # Check length
        # if len(control_data) != 11:
        #     print("length")
        #     return False

        # Check for initial control character
        # TODO
        # if not control_data[0].isalpha():
        #     print("alpha")
        #     return False

        # All checks passed
        return True
