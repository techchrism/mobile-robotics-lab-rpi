import serial
import threading
import time


class ArduinoManager:
    callback = None
    ser = None

    def __init__(self, device, callback):
        self.callback = callback
        self.ser = serial.Serial(device, 115200)

    def start(self):
        t = threading.Thread(target=self.start_async)
        t.start()

    def start_async(self):
        time.sleep(1)
        while True:
            line = self.ser.readline().decode('ascii').strip()
            # Try parsing as 5 ultrasonic data
            parts = line.split(' ')
            if len(parts) == 5:
                try:
                    self.callback([float(s) for s in parts])
                except Exception as e:
                    print('Got error while parsing arduino data')
                    print(e)

    def send_speed(self, v, w):
        data = f'{v} {w}\n'
        self.ser.write(data.encode())
