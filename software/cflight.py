import serial
import time
import csv
import enum
import threading
import struct

DEBUG = True
PING_EVERY = 1000  # ms
PING_TIMEOUT = 5000  # ms



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
    HANDSHAKE = 7,
    ERROR = 8,
    REPORT = 9


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

    @staticmethod
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

    @staticmethod
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
            header.type, header.salt, header.time, list(
                bytes[16: header.length + 16])
        )
        calcCheck = 0
        for b in packet.payload + [0xFF, 0xFE, 0xFB, 0xFF]:
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


class Device:
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

    def sendPacket(self, packet):
        asBytes = packet.encode()
        self.device.write(asBytes)

    def readloop(self):
        print("Read loop started")
        while self.running:
            if self.connected and self.device.in_waiting > 0:
                readline = self.device.readline().decode().strip()
                print("[DEVICE]: ", readline)

        print("Read loop exiting")

    def registerCallback(self, callback):
        self.callback = callback


def unitTest():
    packet = Packet(2, 69, 1000, [x for x in range(102)])
    packet.print()
    print("")
    encoded = packet.encode()

    packet2 = Packet.decode(encoded)
    if (packet2 == None):
        print("Error decoding packet")
    else:
        packet2.print()


class CFlight:
    def __init__(self, commsPort, commsBaud):

        self.deviceStatus = 0  # 0 = Disconnected, 1 = Connected, 2 = Nominal, 3 = Error, 4 = timeout
        self.lastDeviceStatus = 0
        self.lastPong = 0
        self.lastPing = 0
        self.device = Device(commsPort, commsBaud)
        self.device.connect()
        time.sleep(1)
        self.device.registerCallback(self.packetCallback)
        reportCmd = Command(CommandType.SYSTEM_REPORT, [])
        pollStatus = Packet(PacketType.COMMAND, 0, time.time(), reportCmd.encode())
        self.device.sendPacket(pollStatus)
        self.commandQueue = []
        self.callbacks = {}
        self.running = True
        self.threadbox = threading.Thread(target=self.loopyDoopey)
        self.threadbox.start()
    
    def stop(self):
        self.running = False
        self.device.closeConnection()

    def queueCommand(self, command):
        self.commandQueue.append(command)
    
    def sendPacket(self, packet):
        self.device.sendPacket(packet)
    
    def registerCallback(self, callback, callbackType):
        # callback must be a function that takes a packet as an argument
        if callbackType in self.callbacks:
            self.callbacks[callbackType].append(callback)
        else:
            self.callbacks[callbackType] = [callback]


    def parsePong(self, payload):
        # payload 0:4 = ping id (ignore for now) TODO: check ping ID
        pingId = int.from_bytes(payload[0:4], byteorder="little")
        if pingId != self.lastPing:
            print("Warning: pong ID mismatch. Expected {}, got {}".format(self.lastPing, pingId))
        self.lastPong = time.time()


    def packetCallback(self, packetRaw):
        packet = Packet.decode(packetRaw)
        if packet == None:
            print("Error decoding packet")
            return
        
        callbacks = self.callbacks.get(packet.header.type, [])
        for callback in callbacks:
            callback(packet)

        if packet.header.type == PacketType.REPORT:
            self.parseSystemReport(packet.payload)
        elif packet.header.type == PacketType.PONG:
            self.parsePong(packet.payload)


    def parseSystemReport(self, payload):
        # System report starts with "- System Report -"
        reportHeaderEnd = len("- System Report -") + 1
        summaryLine = ""
        for i in range(reportHeaderEnd, len(payload)):
            if payload[i] == '\n':
                summaryLine = payload[reportHeaderEnd:i]
                break

        if summaryLine == "":
            print("Error parsing system report")
            self.deviceStatus = 1
            return False
        if DEBUG:
            print("System report: {}".format(summaryLine))

        # Parse summary line
        # Format: "OS: <OK|ERROR>, Uptime: <uptime>, Errors: <error count> |"
        summary = summaryLine.split(",")
        if len(summary) != 3:
            print("Error parsing system report summary")
            self.deviceStatus = 1
            return False
        OpStatus = summary[0].split(":")[1].strip()
        Uptime = summary[1].split(":")[1].strip()
        Errors = summary[2].split(":")[1].strip()
        if DEBUG:
            print("OpStatus: {}, Uptime: {}, Errors: {}".format(
                OpStatus, Uptime, Errors))

        if OpStatus != "OK" or Uptime == "" or Errors != "0":
            print("Error: system report status is not OK")
            self.deviceStatus = 3
        else:
            self.deviceStatus = 2

    def loopyDoopey(self):
        try:
            currentTime = time.time()
            while self.running:
                if (currentTime - self.lastPing >= PING_EVERY):
                    ping = Packet(PacketType.PING, 0, currentTime,
                                struct.pack("f", currentTime))
                    self.device.sendPacket(ping)
                    self.lastPing = currentTime
                if (currentTime - self.lastPong >= PING_TIMEOUT):
                    print("Error: ping/pong timeout")
                    self.deviceStatus = 4

                if (self.deviceStatus == 2):
                    for cmdIndex in range(len(self.commandQueue)):
                        cmd = self.commandQueue[cmdIndex]
                        if type(cmd) != Command:
                            print(
                                "Error: command queue contains non-command, skipping. Type: {}".format(type(cmd)))
                            continue
                        cmdPacket = Packet(PacketType.COMMAND, 0,
                                        time.time(), cmd.encode())
                        self.device.sendPacket(cmdPacket)
                        self.commandQueue.pop(cmdIndex)

        except KeyboardInterrupt:
            print("Exiting...")
            self.device.closeConnection()
            self.running = False
