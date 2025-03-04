#!/bin/env python3

import serial
import sys
import time

hexfile="../../test072/Debug/test072_crc.hex"
#hexfile="../../test051/Debug/test051_crc.hex"
#hexfile="../../test051/Debug/test051.hex"

hexin= open(hexfile, "r")
try:
    SerName= sys.argv[1]
except:
    print("Needs device as 1. argument")
    exit(0)
print (sys.argv[0], sys.argv[1])

try:
    ser= serial.Serial(port=sys.argv[1], baudrate=115200,
                       bytesize=serial.EIGHTBITS,
                       stopbits=serial.STOPBITS_TWO, timeout = 1)
except:
    print("Serial port %s not opened"%sys.argv[1])

errc= 0
while True:
    l= hexin.readline()
    if l== '': break
    ser.flushOutput()
    ser.flushInput()
    l= l+'\n'
    ser.write(str.encode(l))
    print(l)
    #time.sleep(1)
    rec=''
    while True:
        c= ser.read(1)
        
        # print (rec,c)
        if c == b'\n': break
        rec = rec + str(c, 'UTF-8')
    #rec= ser.read_until('b\n')
    received=  rec.encode("utf-8")
    print("Received: ", received)
    if received != b'00000000': errc += 1
    #input("Press the Enter key to continue: ")
print ("Errors: %d "%errc)      

    

    


