import serial
import time
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
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

print("Loading YOLO models...")
model_general = YOLO("yolo11s.pt")          # COCO (people, cars, etc.)
model_crosswalk = YOLO("yolov8n.pt")         # Your crosswalk model

# Load stereo calibration data
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
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 5,
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
    mask = (region > 1) & (region < 128)  # example thresholds
    
    if np.count_nonzero(mask) == 0:
        return None
    
    disp_valid = region[mask]
    avg_disp = np.median(disp_valid)
    
    if avg_disp <= 0:
        return None
    
    points_3D = cv2.reprojectImageTo3D(disparity_map, Q)
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    center_disp = disparity_map[center_y, center_x]
    
    if center_disp <= 0:
        return None
    
    distance = points_3D[center_y, center_x][2] * 100  # meters to cm
    
    # Only trust distance if it’s reasonable
    if distance < 0 or distance > 5000:  # 50 meters maximum
        return None
    
    return distance

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

        detected_obstacles = []

        # Process general objects (people, cars, etc.)
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

        # Process crosswalks separately
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

            if distance_cm is not None and distance_cm < 500:  # Crosswalks might be further away
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
