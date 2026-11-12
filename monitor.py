import serial,time
ser=serial.Serial('/dev/ttyACM0',460800,timeout=0.5);time.sleep(2)
ser.write(b"AT+ZBJOIN=90\r\n"); time.sleep(1)
start=time.time()
while time.time()-start<95:
    line=ser.readline().decode(errors='ignore').strip()
    if line:
        print(line)
ser.close()
