import serial
import time
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
import numpy as np
import matplotlib.pyplot as plt

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
Q = calib['Q']

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

def capture_and_detect():
    i = 0
    while True:
        print(f"\n--- Capture {i} ---")
        print("Capturing images...")
        imgL = camera1.capture_array()
        imgR = camera2.capture_array()

        imgL_rgb = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
        imgR_rgb = cv2.cvtColor(imgR, cv2.COLOR_BGR2RGB)

        print("Running object detection (YOLO11s)...")
        results_general = model_general(imgL_rgb)[0]

        print("Running crosswalk detection (YOLOv8n)...")
        results_crosswalk = model_crosswalk(imgL_rgb)[0]

        print("Computing depth map...")
        disparity = compute_depth_map(imgL, imgR)

        # Normalize disparity for visualization
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)

        # Apply a color map (JET) to the disparity for better visualization
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        # Update matplotlib plot
        disp_plot.set_data(cv2.cvtColor(disp_color, cv2.COLOR_BGR2RGB))
        fig.canvas.draw()
        fig.canvas.flush_events()

        detected_obstacles = []

        for box in results_general.boxes:
            cls_id = int(box.cls[0])
            label = model_general.names[cls_id]
            conf = float(box.conf[0])
            if conf < 0.5:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            distance_cm = get_object_distance((x1, y1, x2, y2), disparity, Q)

            if distance_cm is None:
                lidar_data = read_tfluna_data()
                if lidar_data:
                    distance_cm = lidar_data["distance"]

            if distance_cm is not None and distance_cm < 200:
                detected_obstacles.append({
                    "label": label,
                    "distance_cm": round(distance_cm, 1)
                })

        for box in results_crosswalk.boxes:
            cls_id = int(box.cls[0])
            label = model_crosswalk.names[cls_id]
            conf = float(box.conf[0])
            if conf < 0.5:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            distance_cm = get_object_distance((x1, y1, x2, y2), disparity, Q)

            if distance_cm is None:
                lidar_data = read_tfluna_data()
                if lidar_data:
                    distance_cm = lidar_data["distance"]

            if distance_cm is not None and distance_cm < 500:
                detected_obstacles.append({
                    "label": label,
                    "distance_cm": round(distance_cm, 1)
                })

        if detected_obstacles:
            print("\nDetected Obstacles:")
            for obs in detected_obstacles:
                print(f"  - {obs['label']}: {obs['distance_cm']} cm")
        else:
            print("No nearby obstacles detected.")

        i += 1
        time.sleep(5)


def main():
    try:
        capture_and_detect()
    except KeyboardInterrupt:
        print("\nStopping program...")
    finally:
        camera1.stop()
        camera2.stop()
        ser.close()
        print("Cameras and LiDAR stopped.")

if __name__ == "__main__":
    main()
