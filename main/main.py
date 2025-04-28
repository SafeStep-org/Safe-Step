import asyncio
import serial
import time
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
import numpy as np
import matplotlib.pyplot as plt
import ble_server

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

print("Loading YOLO models...")
model_general = YOLO("yolo11s.pt")          # COCO (people, cars, etc.)
model_crosswalk = YOLO("yolov8n.pt")         # Your crosswalk model

# Load stereo calibration data
calib = np.load("stereo_calib_data.npz")
mtxL, distL = calib['mtxL'], calib['distL']
mtxR, distR = calib['mtxR'], calib['distR']
R, T = calib['R'], calib['T']

# Get image size
frameL = camera1.capture_array()
frameR = camera2.capture_array()
img_size = (frameL.shape[1], frameL.shape[0])

# Stereo rectify
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, img_size, R, T)

# Precompute rectification maps
mapLx, mapLy = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, img_size, cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, img_size, cv2.CV_32FC1)

# StereoSGBM matcher
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16 * 8,
    blockSize=3,
    P1=8 * 3 * 3 ** 2,
    P2=32 * 3 * 3 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# Setup matplotlib plot
plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
disp_plot = ax.imshow(np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8))
fig.colorbar(disp_plot)
ax.set_title('Disparity Map')
ax.axis('off')
fig.tight_layout()

print("Starting BLE Server...")


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
    rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)
    
    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

    disparity = stereo.compute(grayL, grayR).astype(np.float32) / 16.0
    print(f"{np.min(disparity)} {np.max(disparity)} {np.mean(disparity)}" )
    return disparity

def get_object_distance(bbox, disparity_map, Q):
    x1, y1, x2, y2 = map(int, bbox)
    region = disparity_map[y1:y2, x1:x2]

    valid_disp_min = 1
    valid_disp_max = 128

    mask = (region > valid_disp_min) & (region < valid_disp_max)

    if np.count_nonzero(mask) == 0:
        return None

    disp_valid = region[mask]
    median_disp = np.median(disp_valid)

    points_3D = cv2.reprojectImageTo3D(disparity_map, Q)

    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2

    center_disp = disparity_map[center_y, center_x]

    distance_from_center = None
    if valid_disp_min < center_disp < valid_disp_max:
        point_center = points_3D[center_y, center_x]
        distance_from_center = point_center[2] * 100

    if median_disp > 0:
        temp_disp_map = np.full_like(disparity_map, median_disp)
        points_median = cv2.reprojectImageTo3D(temp_disp_map, Q)
        distance_from_median = points_median[center_y, center_x][2] * 100
    else:
        distance_from_median = None

    distances = []
    if distance_from_center is not None and 0 < distance_from_center < 5000:
        distances.append(distance_from_center)
    if distance_from_median is not None and 0 < distance_from_median < 5000:
        distances.append(distance_from_median)

    if distances:
        return np.median(distances)
    else:
        return None

def capture_and_detect(server):
    i = 0
    while True:
        print(f"\n--- Capture {i} ---")
        print("Capturing images...")
        imgL = camera1.capture_array()
        imgR = camera2.capture_array()

        imgL_rgb = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)

        print("Running object detection (YOLO11s)...")
        results_general = model_general(imgL_rgb)[0]

        print("Running crosswalk detection (YOLOv8n)...")
        results_crosswalk = model_crosswalk(imgL_rgb)[0]

        print("Computing depth map...")
        disparity = compute_depth_map(imgL, imgR)

        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        disp_plot.set_data(cv2.cvtColor(disp_color, cv2.COLOR_BGR2RGB))
        fig.canvas.draw()
        fig.canvas.flush_events()

        detected_objects = []

        valid_disp_min = 1
        valid_disp_max = 128

        points_3D = cv2.reprojectImageTo3D(disparity, Q)

        # Step 1: Analyze all YOLO detections
        for results, model in [(results_general, model_general), (results_crosswalk, model_crosswalk)]:
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                region = disparity[y1:y2, x1:x2]
                mask = (region > valid_disp_min) & (region < valid_disp_max)

                if np.count_nonzero(mask) == 0:
                    continue

                disp_valid = region[mask]
                median_disp = np.median(disp_valid)

                temp_disp_map = np.full_like(disparity, median_disp)
                points_median = cv2.reprojectImageTo3D(temp_disp_map, Q)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance_cm = points_median[center_y, center_x][2] * 100  # meters to cm

                if distance_cm <= 0 or distance_cm > 5000:
                    continue

                detected_objects.append({
                    "label": label,
                    "distance_cm": distance_cm
                })

        # Step 2: If no YOLO detections found, fallback to closest pixel
        if not detected_objects:
            print("No YOLO detections, falling back to closest depth pixel.")
        
            mask_valid = (disparity > valid_disp_min) & (disparity < valid_disp_max)
        
            if not np.any(mask_valid):
                print("No valid disparity points found.")
                i += 1
                time.sleep(5)
                continue
        
            # Find the valid pixel with the minimum Z-distance (closest)
            distances_cm = points_3D[:, :, 2] * 100
            distances_cm_masked = np.where(mask_valid, distances_cm, np.inf)
            min_idx = np.unravel_index(np.argmin(distances_cm_masked), distances_cm_masked.shape)
            center_y, center_x = min_idx
        
            distance_cm = distances_cm[center_y, center_x]
        
            # Reject the border artifacts (~20.7 cm constant)
            if distance_cm <= 21.0:
                print(f"Closest point ({distance_cm:.1f} cm) is likely border noise. Skipping...")
                i += 1
                time.sleep(5)
                continue
        
            if distance_cm <= 0 or distance_cm > 5000:
                print("Depth map distance invalid or too far.")
                i += 1
                time.sleep(5)
                continue
        
            detected_objects.append({
                "label": "obstacle",
                "distance_cm": distance_cm
            })


        # Step 3: Choose the object with the minimum distance
        detected_objects = sorted(detected_objects, key=lambda x: x["distance_cm"])
        closest_object = detected_objects[0]

        # Step 4: Cross-check with LiDAR
        lidar_data = read_tfluna_data()
        if lidar_data:
            lidar_distance_cm = lidar_data["distance"]
            if abs(lidar_distance_cm - closest_object["distance_cm"]) > 100:
                print(f"Depth map distance ({closest_object['distance_cm']:.1f} cm) differs from LiDAR ({lidar_distance_cm:.1f} cm). Using LiDAR.")
                closest_object["distance_cm"] = lidar_distance_cm
            else:
                print("Depth map and LiDAR distances match within tolerance.")

        # Step 5: Report
        print("\nClosest Object Detected:")
        print(f"  Label: {closest_object['label']}")
        print(f"  Distance: {round(closest_object['distance_cm'], 1)} cm")
        server.send_message(f"{closest_object['label']} found {closest_object['distance'] / 100} meters away")

        i += 1
        time.sleep(5)

async def main():
    try:
        loop = asyncio.get_running_loop()
        server = ble_server.SafePiBLEServer(loop)
        await server.start()
 
        print("Waiting for client to write something...")
        if server.trigger.__module__ == "threading":
            await asyncio.to_thread(server.trigger.wait)
        else:
            await server.trigger.wait()
 
        print("Client connected and sent data.")
        await server.send_message("Hi PWA!")
 
        await capture_and_detect(server)
    except KeyboardInterrupt:
        print("\nStopping program...")
    finally:
        camera1.stop()
        camera2.stop()
        ser.close()
        print("Cameras and LiDAR stopped.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down...")
