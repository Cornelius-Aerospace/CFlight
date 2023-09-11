# A simple serial communication, just read user text and send to serial port followed by a single new line

from orhelper import FlightDataType
import orhelper
import os
import serial
import time
import csv

FAKE_DATA = False
port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)


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


simulation = [
]

simIndex = 0


def simulateFlight():
    with orhelper.OpenRocketInstance() as instance:
        orh = orhelper.Helper(instance)

        # Load document, run simulation and get data and events

        doc = orh.load_doc(os.path.join('examples', 'simple.ork'))
        sim = doc.getSimulation(2)
        orh.run_simulation(sim)
        data = orh.get_timeseries(sim, [FlightDataType.TYPE_TIME, FlightDataType.TYPE_ALTITUDE,
                                  FlightDataType.TYPE_AIR_PRESSURE, FlightDataType.TYPE_AIR_TEMPERATURE])
        for t in range(len(data[FlightDataType.TYPE_TIME])):
            entry = SimEntry(int(float(data[FlightDataType.TYPE_TIME][t])*1000), round(data[FlightDataType.TYPE_ALTITUDE][t], 2), round(
                data[FlightDataType.TYPE_AIR_TEMPERATURE][t], 2), int(data[FlightDataType.TYPE_AIR_PRESSURE][t])/100)
            entry.print()
            simulation.append(entry)

        simulation[0].time = 1


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
    print(greetingMsg.decode())

port.write(b"START\n")
while (greetingMsg != b"ACK"):
    greetingMsg = port.readline().strip()
    print(greetingMsg.decode())

doneSending = False
while True:
    # We cant use input() as it will block and we cant use raw_input() as it will not work on python3
    # So we use sys.stdin.readline() which will read from stdin and return a string
    # We strip the string to remove the trailing newline
    # We encode the string to bytes as pyserial expects bytes
    # We know there hasnt been input if the string is empty
    try:
        
        if not doneSending:
            if simIndex < len(simulation):
                # simulation[simIndex].print()
                port.write(simulation[simIndex].formMsg().encode())
                simIndex += 1

            else:
                doneSending = True
                for i in range(0, 5):
                    # Ensure device knows sim ended
                    port.write(
                        SimEntry(0, 0, 0, simulation[0].pressure).formMsg().encode())
        while port.in_waiting > 0:
            read = port.readline()
            try:
                if read:
                    strread = read.decode().strip()
                    print(strread)
                    if strread == "End of report":
                        print("Flight complete, bye")
                        port.close()
                        exit()
            except Exception as e:
                print(e)
        time.sleep(10/1000)
    except KeyboardInterrupt:
        print("Ctrl-c pressed, bye...")
        port.close()
        break
