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
import atexit
import threading
import queue

display_queue = queue.Queue(maxsize=2)
stop_display_event = threading.Event()

# === Initialize hardware ===
print("Initializing cameras...")
camera1 = Picamera2(0)
camera2 = Picamera2(1)
camera1.set_controls({"AfMode": controls.AfModeEnum.Continuous})
camera2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
camera1.start()
camera2.start()
print("Cameras started.")

print("Initializing TF-Luna LiDAR...")
ser = serial.Serial("/dev/ttyAMA0", 115200)

print("Loading YOLO model...")
model_general = YOLO("yolo11s.pt")  # Single YOLO model for general detection

# === Stereo Calibration ===
calib = np.load("stereo_calib_data.npz")
mtxL, distL = calib['mtxL'], calib['distL']
mtxR, distR = calib['mtxR'], calib['distR']
R, T = calib['R'], calib['T']

# Get image size
frameL = camera1.capture_array()
frameR = camera2.capture_array()
img_size = (frameL.shape[1], frameL.shape[0])

R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, img_size, R, T)
mapLx, mapLy = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, img_size, cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, img_size, cv2.CV_32FC1)

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16 * 7,
    blockSize=5,
    P1=8 * 3 * 3 ** 2,
    P2=32 * 3 * 3 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# === Setup plot ===
# === Setup Matplotlib Display ===
plt.ion()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
disp_plot = ax1.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
cam_plot = ax2.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
ax1.set_title("Depth Map")
ax2.set_title("YOLO Detections")


# === LiDAR Reader ===
def read_tfluna_data():
    if ser.in_waiting > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()
        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
            distance = bytes_serial[2] + bytes_serial[3] * 256
            strength = bytes_serial[4] + bytes_serial[5] * 256
            temperature = (bytes_serial[6] + bytes_serial[7] * 256) / 8.0 - 256.0
            return {"distance": distance, "strength": strength, "temperature": temperature}
    return None


# === Depth Map Computation ===
def compute_depth_map(imgL, imgR):
    rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)
    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(grayL, grayR).astype(np.float32) / 16.0
    return disparity


# === BLE Callback ===
async def take_picture():
    img = camera1.capture_array()
    cv2.imwrite(f'pictures/{time.time()}.jpg', img)
    await asyncio.sleep(0)

def display_thread_func():
    while not stop_display_event.is_set():
        try:
            disp_color, annotated_img = display_queue.get(timeout=1)

            disp_plot.set_data(cv2.cvtColor(disp_color, cv2.COLOR_BGR2RGB))
            cam_plot.set_data(cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB))
            fig.canvas.draw()
            fig.canvas.flush_events()

        except queue.Empty:
            continue

# === Main Detection Loop with Optimizations ===
async def capture_and_detect(server: ble_server.SafePiBLEServer):
    i = 0
    frame_skip = 3
    last_annotated_img = np.zeros((480, 640, 3), dtype=np.uint8)
    last_disp_color = np.zeros((480, 640, 3), dtype=np.uint8)

    while True:
        print(f"\n--- Frame {i} ---")
        imgL = camera1.capture_array()
        imgR = camera2.capture_array()
        imgL_rgb = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)

        if i % frame_skip == 0:
            print("Running YOLO11s detection...")
            results = model_general(imgL_rgb)[0]
            await asyncio.sleep(0)

            print("Computing depth map...")
            disparity = compute_depth_map(imgL, imgR)
            await asyncio.sleep(0)

            disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_vis = np.uint8(disp_vis)
            disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        
            if not display_queue.full():
                display_queue.put((disp_color, annotated_img))

            # Distance estimation
            detected_objects = []
            valid_disp_min, valid_disp_max = 1, 128
            points_3D = cv2.reprojectImageTo3D(disparity, Q)

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                label = model_general.names[cls_id]

                region = disparity[y1:y2, x1:x2]
                mask = (region > valid_disp_min) & (region < valid_disp_max)
                if not np.any(mask):
                    continue

                median_disp = np.median(region[mask])
                if median_disp <= 0:
                    continue

                temp_disp_map = np.full_like(disparity, median_disp)
                points_median = cv2.reprojectImageTo3D(temp_disp_map, Q)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance_cm = points_median[center_y, center_x][2] * 100

                if 0 < distance_cm < 5000:
                    detected_objects.append({"label": label, "distance_cm": distance_cm})

            if not detected_objects:
                print("No YOLO detections, checking closest disparity pixel...")
                mask_valid = (disparity > valid_disp_min) & (disparity < valid_disp_max)
                if np.any(mask_valid):
                    distances_cm = points_3D[:, :, 2] * 100
                    distances_cm_masked = np.where(mask_valid, distances_cm, np.inf)
                    min_idx = np.unravel_index(np.argmin(distances_cm_masked), distances_cm_masked.shape)
                    distance_cm = distances_cm[min_idx]
                    if 21.0 < distance_cm < 5000:
                        detected_objects.append({"label": "obstacle", "distance_cm": distance_cm})

            if detected_objects:
                detected_objects.sort(key=lambda x: x["distance_cm"])
                closest_object = detected_objects[0]

                lidar_data = read_tfluna_data()
                if lidar_data:
                    lidar_distance = lidar_data["distance"]
                    if abs(lidar_distance - closest_object["distance_cm"]) > 100:
                        print(f"LiDAR discrepancy ({lidar_distance} cm), overriding.")
                        closest_object["distance_cm"] = lidar_distance

                print(f"→ Closest: {closest_object['label']} @ {closest_object['distance_cm']:.1f} cm")
                await server.send_message(f"{closest_object['label']} found {closest_object['distance_cm'] / 100:.1f} meters away")

        # Update plots on every frame
        disp_plot.set_data(cv2.cvtColor(last_disp_color, cv2.COLOR_BGR2RGB))
        cam_plot.set_data(cv2.cvtColor(last_annotated_img, cv2.COLOR_BGR2RGB))
        fig.canvas.draw()
        fig.canvas.flush_events()

        i += 1
        await asyncio.sleep(0)

# === Main Entrypoint ===
async def main():
    try:
        loop = asyncio.get_running_loop()
        display_thread = threading.Thread(target=display_thread_func, daemon=True)
        display_thread.start()
        server = ble_server.SafePiBLEServer(loop)
        server.register_callback(take_picture)
        await server.start()
        await capture_and_detect(server)
    except KeyboardInterrupt:
        print("\nInterrupted. Shutting down...")
    finally:
        camera1.stop()
        camera2.stop()
        ser.close()
        atexit.register(lambda: plt.close('all'))
        stop_display_event.set()
        display_thread.join()
        print("Cameras and LiDAR stopped.")

if __name__ == "__main__":
    asyncio.run(main())
