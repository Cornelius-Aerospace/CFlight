import serial
import time
import csv
import enum
import threading


class SimEntry:
    def __init__(self, time, alt, temp, pressure):
        self.time = time
        self.alt = alt
        self.temp = temp
        self.pressure = pressure

    def formMsg(self):
        message = "{},{},{}\n".format(self.time, self.pressure, self.temp)
        return message

    def print(self):
        print(
            "Entry() - Time: {} ms, Alt: {}m, Temp: {}, Air pressure: {} mbar".format(
                self.time, self.alt, self.temp, self.pressure
            )
        )


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


class PacketType(enum.Enum):
    PING = 0
    PONG = 1
    TELEMETRY = 2
    COMMAND = 3
    SETTING = 4
    ACK = 5
    CODE = 6
    HANDSHAKE = 7


class PacketHeader:
    def __init__(self, packetType=0, payloadLength=0, salt=0, time=0):
        self.type = packetType
        self.length = payloadLength
        self.salt = salt
        self.time = time

    def print(self, end="\n"):
        print(
            "PH: type={}, length={}, salt={}, time={}".format(
                self.type, self.length, self.salt, self.time
            ),
            end=end,
        )

    def encode(self):
        encoded = bytearray([0xFF, 0xFF, 0xFF])  # Sync bytes
        encoded.extend(self.time.to_bytes(4, "little"))
        encoded.append(self.type)
        encoded.append(0)
        encoded.append(self.length)
        encoded.extend(self.salt.to_bytes(4, "little"))
        encoded.extend([0xFF, 0xFF])
        return encoded

    def decode(bytes):
        if bytes[0] + bytes[1] + bytes[2] != 255 * 3:
            print("Error in packet header, missing sync bytes (start)")
            return None
        header = PacketHeader()
        header.time = int.from_bytes(bytes[3:7], "little")
        header.type = int(bytes[7])
        header.length = int(bytes[9])
        header.salt = int.from_bytes(bytes[10:14], "little")
        if bytes[14] + bytes[15] != 255 * 2:
            print(
                "Error in packet header missing sync bytes (end) [14: {} 15: {}]".format(
                    bytes[14], bytes[15]
                )
            )
            return None

        return header


class Packet:
    def __init__(self, packetType, salt, time, payload):
        self.header = PacketHeader(packetType, len(payload), salt, time)
        self.payload = payload

    def encode(self):
        encoded = self.header.encode()
        encoded.extend(self.payload)
        encoded.extend([0xFF, 0xFE, 0xFB, 0xFF])
        checksum = 0
        i = 0
        for b in encoded:
            if i >= 16:
                checksum += b
                checksum %= 255
            i += 1
        encoded[8] = checksum
        return encoded

    def decode(bytes):
        header = PacketHeader.decode(bytes)
        if header == None:
            return None
        if header.length + 4 + 16 != len(bytes):
            print(
                "Warning: passed bytes array longer than header + claimed payload length ({} != {})".format(
                    len(bytes), 16 + header.length + 4
                )
            )

        packet = Packet(
            header.type, header.salt, header.time, bytes[16 : header.length + 16]
        )
        calcCheck = 0
        for b in packet.payload + bytearray([0xFF, 0xFE, 0xFB, 0xFF]):
            calcCheck += b
            calcCheck %= 255
        if calcCheck != bytes[8]:
            print(
                "Warning: checksum mismatch. Calculated: {}, In-header: {}".format(
                    calcCheck, bytes[8]
                )
            )
        return packet

    def print(self, end="\n"):
        print("Packet: ", end="")
        self.header.print(end="; Payload: ")
        print(self.payload, end=end)


class Command:
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


class CFlight:
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
            if self.connected and self.device.in_waiting > 0:
                readline = self.device.readline().decode().strip()
                print("[DEVICE]: ", readline)

        print("Read loop exiting")


packet = Packet(2, 69, 1000, [x for x in range(102)])
i = 0
encoded = packet.encode()
print("Encoded bytes")
for b in encoded:
    print("{}: {}".format(i, b))
    i += 1
print("")
packet2 = Packet.decode(encoded)
packet2.print()

"""
device = CFlight()
device.connect()
time.sleep(1)
armCmd = Command(CommandType.SET_FLIGHT, [1, 1, 200])
device.sendCommand(armCmd)
time.sleep(0.5)
armCmd = Command(CommandType.ARM, [])
device.sendCommand(armCmd)
input("Press enter to exit: \n")
device.closeConnection()
"""
