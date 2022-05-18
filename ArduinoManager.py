import time
import serial_asyncio


class ArduinoManager:
    callback = None
    reader = None
    writer = None
    device = None

    def __init__(self, device, callback):
        self.callback = callback
        self.device = device

    async def start(self):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=self.device, baudrate=115200)
        while True:
            line = await self.reader.readline()
            line = line.decode('ascii').strip()
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
        self.writer.write(data.encode())
