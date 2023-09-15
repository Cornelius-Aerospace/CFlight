import serial
import time
import csv
import enum
import threading


class SimEntry():
    def __init__(self, time, alt, temp, pressure):
        self.time = time
        self.alt = alt
        self.temp = temp
        self.pressure = pressure

    def formMsg(self):
        message = "{},{},{}\n".format(
            self.time, self.pressure, self.temp)
        return message

    def print(self):
        print("Entry() - Time: {} ms, Alt: {}m, Temp: {}, Air pressure: {} mbar".format(
            self.time, self.alt, self.temp, self.pressure))


class CommandType(enum.Enum):
    ARM = 0
    UNARM = 1
    SET_FLIGHT = 2
    SYSTEM_REPORT = 3
    SET_BUZZER = 4
    BATTERY_CHECK = 5
    SLEEP = 6
    POWER_DOWN = 7
    READ_FLIGHT = 8


class Command():
    def __init__(self, type, arguments):
        self.type = type
        self.arguments = arguments

    def argstring(self):
        asstr = ""
        for arg in self.arguments:
            asstr += str(arg)
            asstr += ","

        return asstr[:-1]

    def encode(self):
        return "{}:{};\n".format(self.type.value, self.argstring()).encode()


class CFlight():
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.port = port
        self.baud = baud
        self.device = None  # type: serial.Serial
        self.connected = False
        self.running = False
        self.threadbox = None

    def connect(self):
        print("Connecting to {} @ {} baud".format(self.port, self.baud))
        self.device = serial.Serial(self.port, baudrate=self.baud, timeout=3.0)
        print("Connected!")
        self.connected = True
        self.running = True
        self.threadbox = threading.Thread(target=self.readloop)
        self.threadbox.start()

    def closeConnection(self):
        self.running = False
        time.sleep(0.1)
        if self.connected:
            self.device.close()
            self.connected = False

    def sendCommand(self, command):
        asBytes = command.encode()
        self.device.write(asBytes)

    def readloop(self):
        print("Read loop started")
        while self.running:
            if (self.connected and self.device.in_waiting > 0):
                readline = self.device.readline().decode().strip()
                print("[DEVICE]: ", readline)

        print("Read loop exiting")


device = CFlight()
device.connect()
time.sleep(1)
armCmd = Command(CommandType.SET_FLIGHT, [1,1,200])
device.sendCommand(armCmd)
time.sleep(0.5)
armCmd = Command(CommandType.ARM, [])
device.sendCommand(armCmd)
input("Press enter to exit: \n")
device.closeConnection()
