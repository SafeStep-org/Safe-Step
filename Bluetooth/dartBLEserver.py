import asyncio
from bleak import BleakGATTCharacteristic, BleakGATTService, BleakAdvertiser, BleakServer
from bleak.backends.characteristic import GATTCharacteristicProperties
import random

# Define UUIDs
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
OBJECT_UUID = "12345678-1234-5678-1234-56789abcdef1"
DIST_UUID = "12345678-1234-5678-1234-56789abcdef2"

# Mock data generator
def get_sensor_data():
    obj_type = random.choice(["person", "car", "bicycle"])
    dist = round(random.uniform(0.5, 5.0), 2)
    return obj_type, str(dist)

async def run_server():
    server = BleakServer()

    # Define services and characteristics
    service = BleakGATTService(SERVICE_UUID)

    char_obj = BleakGATTCharacteristic(
        OBJECT_UUID,
        [GATTCharacteristicProperties.READ, GATTCharacteristicProperties.NOTIFY]
    )
    char_dist = BleakGATTCharacteristic(
        DIST_UUID,
        [GATTCharacteristicProperties.READ, GATTCharacteristicProperties.NOTIFY]
    )

    service.add_characteristic(char_obj)
    service.add_characteristic(char_dist)
    server.add_service(service)

    await server.start()
    print("BLE server started and advertising...")

    try:
        while True:
            obj, dist = get_sensor_data()
            print(f"Sending: {obj}, {dist}m")

            char_obj.set_value(obj.encode())
            char_dist.set_value(dist.encode())

            await asyncio.sleep(2)
    except KeyboardInterrupt:
        await server.stop()
        print("BLE server stopped.")

if __name__ == "__main__":
    asyncio.run(run_server())
