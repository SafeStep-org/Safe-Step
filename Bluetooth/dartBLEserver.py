import asyncio
from bleak import BleakGATTCharacteristic, BleakGATTService, BleakAdvertiser, BleakServer
from bleak.backends.characteristic import GATTCharacteristicProperties
import random

# Define UUIDs
SERVICE_UUID = "302c754d-63c1-4c28-a5ff-ad3e9f332226"
OBJECT_UUID = "3a98b215-2971-4c6d-b5c2-02597ae99d0e"
DIST_UUID = "3bce9b52-c356-4afb-becd-c7ee0b52d550"

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
