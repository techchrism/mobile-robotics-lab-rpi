from TelemetryManager import TelemetryManager
import asyncio


async def main():
    telemetry = TelemetryManager(9090)
    await telemetry.start()
    await asyncio.Future()


if __name__ == '__main__':
    asyncio.run(main())
