# main.py
import asyncio
from ble_server import BLEServer

async def main():
    loop = asyncio.get_event_loop()
    ble = BLEServer(loop)

    await ble.start()
    await ble.send("Hello from server!")
    await asyncio.sleep(2)
    await ble.stop()

if __name__ == "__main__":
    asyncio.run(main())
