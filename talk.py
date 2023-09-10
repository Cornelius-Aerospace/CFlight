# A simple serial communication, just read user text and send to serial port followed by a single new line

from orhelper import FlightDataType, FlightEvent
import orhelper
import os
import serial
import time
import csv

FAKE_DATA = False
port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)


class SimEntry():
    def __init__(self, time, alt, long, lat, pressure):
        self.time = time
        self.alt = alt
        self.long = long
        self.lat = lat
        self.pressure = pressure

    def formMsg(self):
        message = "{},{},{},{}\n".format(
            self.time, self.pressure, self.lat, self.long)
        return message

    def print(self):
        print("Entry() - Time: {} ms, Alt: {}m, Lat: {}, Long: {}, Air pressure: {} mbar".format(
            self.time, self.alt, self.lat, self.long, self.pressure))


simulation = [
]

simIndex = 0


def simulateFlight():
    with orhelper.OpenRocketInstance() as instance:
        orh = orhelper.Helper(instance)

        # Load document, run simulation and get data and events

        doc = orh.load_doc(os.path.join('examples', 'simple.ork'))
        sim = doc.getSimulation(0)
        orh.run_simulation(sim)
        data = orh.get_timeseries(sim, [FlightDataType.TYPE_TIME, FlightDataType.TYPE_ALTITUDE,
                                  FlightDataType.TYPE_AIR_PRESSURE, FlightDataType.TYPE_LONGITUDE, FlightDataType.TYPE_LATITUDE])
        print("{} {} {} {} {}".format(len(data[FlightDataType.TYPE_TIME]), len(data[FlightDataType.TYPE_ALTITUDE]), len(
            data[FlightDataType.TYPE_LATITUDE]), len(data[FlightDataType.TYPE_LONGITUDE]), len(data[FlightDataType.TYPE_AIR_PRESSURE])))
        print(data[FlightDataType.TYPE_TIME])
        print(data[FlightDataType.TYPE_ALTITUDE])
        for t in range(len(data[FlightDataType.TYPE_TIME])):

            entry = SimEntry(int(float(data[FlightDataType.TYPE_TIME][t])*1000), round(data[FlightDataType.TYPE_ALTITUDE][t], 2), round(data[FlightDataType.TYPE_LONGITUDE][t], 2),
                             round(data[FlightDataType.TYPE_LATITUDE][t], 2), int(data[FlightDataType.TYPE_AIR_PRESSURE][t])/100)
            entry.print()
            simulation.append(entry)


def loadSimulationData(filename="simulation.csv"):
    with open(filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            entry = SimEntry(
                int(float(row[0])*1000), round(float(row[1]), 2), round(float(row[2]), 2), round(float(row[3]), 2), 0)
            entry.print()
            simulation.append(entry)


simulateFlight()
print("Loaded simulation.csv")
greetingMsg = ""
while (greetingMsg != b"Sim mode: awaiting packets"):
    greetingMsg = port.readline().strip()
    print(greetingMsg)

port.write(b"START\n")
while (greetingMsg != b"ACK"):
    greetingMsg = port.readline().strip()
    print(greetingMsg)

sendTicker = 0

report = []
capturingReport = False
while True:
    # We cant use input() as it will block and we cant use raw_input() as it will not work on python3
    # So we use sys.stdin.readline() which will read from stdin and return a string
    # We strip the string to remove the trailing newline
    # We encode the string to bytes as pyserial expects bytes
    # We know there hasnt been input if the string is empty
    try:
        while port.in_waiting > 0:
            read = port.readline()
            try:
                if read:
                    strread = read.decode().strip()
                    if capturingReport:
                        if strread != "End of report":
                            report.append(strread + "\n")
                        else:
                            with open("flight.csv", "w+") as file:
                                file.writelines(report)
                            print("wrote report to disk")
                            quit()

                    else:
                        print(strread)
                        if strread == "-- Data --":
                            capturingReport = True
            except Exception as e:
                print(e)
        if sendTicker >= 1:
            if simIndex < len(simulation):
                # simulation[simIndex].print()
                port.write(simulation[simIndex].formMsg().encode())
                simIndex += 1
                sendTicker = 0
        else:
            sendTicker += 1

        time.sleep(10/1000)
    except KeyboardInterrupt:
        print("Ctrl-c pressed, bye...")
        port.close()
        break
