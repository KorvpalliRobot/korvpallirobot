import serial
from serial.tools import list_ports
import queue


class Mainboard:
    # Class to communicate with the mainboard.
    # The only class to have direct access to it.

    def __init__(self):
        # Initialize the serial port
        self.ser = Mainboard.get_mainboard_serial_port()
        # Queues to hold the information going to and coming from the mainboard
        self._to_mainboard = queue.Queue()
        self._from_mainboard = queue.Queue()

    @staticmethod
    # Scan for mainboard serial ports
    def get_mainboard_serial_port():
        ports = list_ports.comports()
        for port in ports:
            try:
                ser = serial.Serial(port.device, 115200, timeout=0.01)
                return ser
            except:
                continue
        raise Exception("Could not find suitable or any USB ports.")

    # COMMUNICATION FROM OTHER CLASSES
    # Generic method to send any string to mainboard
    def send(self, message):
        self._to_mainboard.put(message)

    # Method to send motor speeds to mainboard
    def send_motors(self, motors):
        message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
            round(motors[2])) + ":0\n").encode("'utf-8")
        self._to_mainboard.put(message)

        # Read the returned message.
        self.poll_mainboard()

    def send_thrower(self, thrower_speed):
        message = ("d:" + str(round(thrower_speed)) + "\n").encode("'utf-8")
        self._to_mainboard.put(message)

    # ACTUAL SERIAL COMMUNICATION
    # Method to communicate with the mainboard
    # Should be run in an endless loop??
    def send_to_mainboard(self):
        command = self._to_mainboard.get()
        self.ser.write(command)

    # This method in an endless loop??
    def poll_mainboard(self):
        line = ""
        char = self.ser.read().decode()
        while char != "\n":
            line += char
            char = self.ser.read().decode()

        # Print out the returned message and also return it for other usage.
        self._from_mainboard.put(line)
        return


