import natnet
import asyncio


class OptitrackManager:
    callback = None
    client = None

    def __init__(self, callback):
        self.callback = callback

    async def start(self):
        print('Optitrack startup')
        await asyncio.sleep(0.5)
        print('Optitrack running')
        self.client = natnet.Client.connect()
        print('Connected')
        self.client.set_callback(self._callback)
        await self._spin()

    async def _spin(self):
        #while True:
        #    print('Spinning')
        #    await asyncio.get_event_loop().run_in_executor(None, self.client.run_once)
        await asyncio.get_event_loop().run_in_executor(None, self._full_spin)

    def _full_spin(self):
        self.client.spin()

    def _callback(self, rigid_bodies, markers, timing):
        if self.callback is not None:
            self.callback(rigid_bodies, markers, timing)
