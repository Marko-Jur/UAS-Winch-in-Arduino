import serial
ser = serial.Serial(port = 'COM3')
ser_bytes = ser.readline()