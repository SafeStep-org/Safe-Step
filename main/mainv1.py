
import asyncio
import serial
import time
import torch
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
from ble_server import SafePiBLEServer

# Initialize hardware
print("Initializing cameras...")
camera1 = Picamera2(0)
camera2 = Picamera2(1)

camera1.set_controls({"AfMode": controls.AfModeEnum.Continuous})
camera2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

camera1.start(show_preview=True)
camera2.start(show_preview=True)
print("Cameras initialized and started.")

print("Initializing TF-Luna LiDAR...")
ser = serial.Serial("/dev/ttyAMA0", 115200)

print("Loading YOLO11s model...")
model = YOLO("yolo11s.pt")

def read_tfluna_data():
    time.sleep(1)
    output = {}
    counter = ser.in_waiting
    if counter > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()

        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
            distance = bytes_serial[2] + bytes_serial[3] * 256
            strength = bytes_serial[4] + bytes_serial[5] * 256
            temperature = bytes_serial[6] + bytes_serial[7] * 256
            temperature = (temperature / 8.0) - 256.0
            output = {
                "distance": distance,
                "strength": strength,
                "temperature": temperature
            }
    return output

async def capture_and_detect(server):
    i = 0
    while True:
        print("Capturing images...")
        img1 = camera1.capture_array()
        img2 = camera2.capture_array()

        img1_rgb = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
        img2_rgb = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

        print("Running object detection...")
        results1 = model(img1_rgb)
        results2 = model(img2_rgb)

        print("Camera 1 Results:", results1)
        print("Camera 2 Results:", results2)

        lidar_data = await asyncio.to_thread(read_tfluna_data)
        if lidar_data:
            print("LiDAR:", lidar_data)

        await server.send_message(f'"speak": "Detection {i}: {lidar_data}"')
        i += 1
        await asyncio.sleep(5)

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
    await server.send_message("Hi PWA!")

    # Run detection and sensor loop concurrently
    await capture_and_detect(server)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping program...")
    finally:
        camera1.stop()
        camera2.stop()
        ser.close()
        print("Cameras and LiDAR stopped.")
