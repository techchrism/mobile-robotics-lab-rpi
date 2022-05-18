import json
import time

import websockets
from websockets import serve


class TelemetryManager:
    server = None
    port = 0
    connections = set()
    frame = {}
    frame_id = 0

    def __init__(self, port=8080):
        self.port = port

    async def start(self):
        self.server = await serve(self._connect, 'localhost', self.port)
        print(f'Started telemetry server on port {self.port}')

    def send_frame(self):
        self.frame_id += 1
        self.frame['frame_id'] = self.frame_id
        self.frame['sent_time'] = round(time.time() * 1000)

        websockets.broadcast(self.connections, json.dumps(self.frame))
        self.frame = {}

    async def _connect(self, websocket):
        print('Telemetry connection')
        self.connections.add(websocket)
        try:
            await websocket.wait_closed()
            print('Telemetry disconnection')
        finally:
            self.connections.remove(websocket)
