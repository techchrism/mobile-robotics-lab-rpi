from websockets import serve


class TelemetryManager:
    server = None
    port = 0
    connections = set()

    def __init__(self, port=8080):
        self.port = port

    async def start(self):
        self.server = serve(self._connect, 'localhost', self.port)
        print(f'Started server on port {self.port}')
        await self.server

    async def _connect(self, websocket):
        print('Websocket connection!')
        self.connections.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.connections.remove(websocket)
