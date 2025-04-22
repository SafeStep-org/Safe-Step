import asyncio
import serial
import time
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
from ble_server import SafePiBLEServer  # Ensure this module exists
import numpy as np

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

# Load stereo calibration data (you must have run calibration before)
calib = np.load("stereo_calib_data.npz")
Q = calib["Q"]

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

def compute_depth_map(imgL, imgR):
    # You can tune these parameters later for better quality
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 5,  # must be divisible by 16
        blockSize=5,
        P1=8 * 3 * 5 ** 2,
        P2=32 * 3 * 5 ** 2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(grayL, grayR).astype(np.float32) / 16.0
    return disparity

def get_object_distance(bbox, disparity_map, Q):
    x1, y1, x2, y2 = map(int, bbox)
    region = disparity_map[y1:y2, x1:x2]
    mask = region > 0
    if np.count_nonzero(mask) == 0:
        return None
    disp_valid = region[mask]
    avg_disp = np.median(disp_valid)
    if avg_disp <= 0:
        return None
    points_3D = cv2.reprojectImageTo3D(disparity_map, Q)
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    return points_3D[center_y, center_x][2] * 100  # meters to cm

async def capture_and_detect(server):
    i = 0
    while True:
        print("Capturing images...")
        imgL = camera1.capture_array()
        imgR = camera2.capture_array()

        imgL_rgb = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
        imgR_rgb = cv2.cvtColor(imgR, cv2.COLOR_BGR2RGB)

        print("Running object detection...")
        resultsL = model(imgL_rgb)[0]

        print("Computing depth map...")
        disparity = compute_depth_map(imgL, imgR)

        detected_obstacles = []

        for box in resultsL.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = float(box.conf[0])
            if conf < 0.5:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            distance_cm = get_object_distance((x1, y1, x2, y2), disparity, Q)

            # Fallback to LiDAR if no stereo data
            if distance_cm is None:
                lidar_data = await asyncio.to_thread(read_tfluna_data)
                if lidar_data:
                    distance_cm = lidar_data["distance"]

            if distance_cm is not None and distance_cm < 200:  # within 2 meters
                detected_obstacles.append({
                    "label": label,
                    "distance_cm": round(distance_cm, 1)
                })

        print("Detected obstacles:", detected_obstacles)

        if detected_obstacles:
            message = {"obstacles": detected_obstacles}
            await server.send_message(str(message).replace("'", '"'))  # JSON-ish string
        else:
            await server.send_message('"status": "No nearby obstacles"')

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
