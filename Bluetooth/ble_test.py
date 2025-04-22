# main.py
import asyncio
from main.ble_server import SafePiBLEServer

async def main():
    loop = asyncio.get_running_loop()
    server = SafePiBLEServer(loop)

    await server.start()

    print("Waiting for client to write something...")
    if server.trigger.__module__ == "threading":
        await asyncio.to_thread(server.trigger.wait)
    else:
        await server.trigger.wait()

    print("Client connected and sent data.")

    # Send a message back
    await server.send_message("Hi PWA!")

    # Wait and send another message every few seconds
    i = 0
    while True:
        await asyncio.sleep(5)
        i += 1
        await server.send_message(f'I love you {i}')

if __name__ == "__main__":
    asyncio.run(main())
