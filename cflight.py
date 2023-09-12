import serial
import time
import csv
import enum

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

class CommandType(enum):
    ARM = 0,
    UNARM = 1,
    SET_FLIGHT = 2,
    SYSTEM_REPORT = 3,
    SET_BUZZER = 4,
    BATTERY_CHECK = 5,
    SLEEP = 6,
    POWER_DOWN = 7,
    READ_FLIGHT = 8,

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
        return "{}:{};".format(str(self.type.value), self.argstring()).encode()


class CFlight():
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.port = port 
        self.baud = baud
        self.device = None # type: serial.Serial
        self.connected = False
    
    def connect(self):
        self.device = serial.Serial(self.port, baudrate=self.baud, timeout=3.0)
        self.connected = True
    
    def closeConnection(self):
        if self.connected:
            self.device.close()
            self.connected = False

    def sendCommand(self, command):
        self.device.write(command.encode())