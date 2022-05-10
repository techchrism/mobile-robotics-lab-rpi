import natnet
import threading


class OptitrackManager:
    callback = None
    client = None

    def __init__(self, callback):
        self.callback = callback

    def start(self):
        t = threading.Thread(target=self.start_async)
        t.start()

    def start_async(self):
        self.client = natnet.Client.connect()
        self.client.set_callback(self._callback)
        self.client.spin()

    def _callback(self, rigid_bodies, markers, timing):
        if self.callback is not None:
            self.callback(rigid_bodies, markers, timing)
