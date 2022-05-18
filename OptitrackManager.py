import natnet
import asyncio


class OptitrackManager:
    callback = None
    client = None

    def __init__(self, callback):
        self.callback = callback

    async def start(self):
        self.client = natnet.Client.connect()
        self.client.set_callback(self._callback)
        await self._spin()

    async def _spin(self):
        while True:
            await asyncio.get_event_loop().run_in_executor(None, self.client.run_once)

    def _callback(self, rigid_bodies, markers, timing):
        if self.callback is not None:
            self.callback(rigid_bodies, markers, timing)
