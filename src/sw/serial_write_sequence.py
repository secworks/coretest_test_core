#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=======================================================================
#
# serial_write_sequence.py
# ------------------------
# Program that sends a sequence of commands to a specific address.
# any response.
#
# Note: This proram requires the PySerial module.
# http://pyserial.sourceforge.net/
#
# 
# Author: Joachim Strömbergson
# Copyright (c) 2014  Secworks Sweden AB
# 
# Redistribution and use in source and binary forms, with or 
# without modification, are permitted provided that the following 
# conditions are met: 
# 
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer. 
# 
# 2. Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in 
#    the documentation and/or other materials provided with the 
#    distribution. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#=======================================================================
 
#-------------------------------------------------------------------
# Python module imports.
#-------------------------------------------------------------------
import sys
import serial
import os
import time

 
#-------------------------------------------------------------------
# Defines.
#-------------------------------------------------------------------
VERBOSE = True

    
#-------------------------------------------------------------------
# main()
#
# Parse arguments.
#-------------------------------------------------------------------
def main():
    # Open device
    ser = serial.Serial()
    ser.port='/dev/cu.usbserial-A801SA6T'
    ser.baudrate=9600
    ser.bytesize=8
    ser.parity='N'
    ser.stopbits=1
    ser.timeout=1
    ser.writeTimeout=0
    ser.open()

    # Send the command sequence.
    for i in range (256):
        my_cmd = ['\x55', '\x11', '\x01', '\x20', '\x00', '\x00', '\x00']
        my_cmd.append(chr(i))
        my_cmd.append('\xaa')
        for tx_byte in my_cmd:
            ser.write(tx_byte)
            time.sleep(0.05)

    ser.close()

    # Exit nicely.
    if VERBOSE:
        print "Done. Closing device."


#-------------------------------------------------------------------
# __name__
# Python thingy which allows the file to be run standalone as
# well as parsed from within a Python interpreter.
#-------------------------------------------------------------------
if __name__=="__main__": 
    # Run the main function.
    sys.exit(main())

#=======================================================================
# EOF serial_write_sequence.py
#=======================================================================
