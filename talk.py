# A simple serial communication, just read user text and send to serial port followed by a single new line

import serial, sys

port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)
while True:
    # We cant use input() as it will block and we cant use raw_input() as it will not work on python3
    # So we use sys.stdin.readline() which will read from stdin and return a string
    # We strip the string to remove the trailing newline
    # We encode the string to bytes as pyserial expects bytes
    # We know there hasnt been input if the string is empty
    s = sys.stdin.readline().strip().encode()
    if s:
        port.write(s + b'\n')
    
    read = port.readline()
    if read:
        print(read.decode().strip())
    
        