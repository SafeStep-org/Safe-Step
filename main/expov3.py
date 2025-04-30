import asyncio
import serial
import time
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO
import numpy as np
import ble_server
import atexit
import threading
import queue

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
    numDisparities=16 * 4,  # was 16*7
    blockSize=5,
    P1=8 * 3 * 3 ** 2,
    P2=32 * 3 * 3 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=50,  # reduced from 100
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM
)

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

# === Main Detection Loop with Optimizations ===
async def capture_and_detect(server: ble_server.SafePiBLEServer):
    i = 0
    frame_skip = 3
    valid_disp_min, valid_disp_max = 1, 128

    while True:
        print(f"\n--- Frame {i} ---")
        imgL = camera1.capture_array()
        imgR = camera2.capture_array()
        imgL_rgb = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)

        annotated_img = imgL.copy()
        disp_color = np.zeros_like(imgL)

        if i % frame_skip == 0:
            # === Run YOLO and depth map in parallel ===
            async def run_yolo():
                small = cv2.resize(imgL_rgb, (416, 416))
                results = model_general(small)[0]
                # Scale boxes back to original resolution
                scale_x = imgL.shape[1] / 416
                scale_y = imgL.shape[0] / 416
                for box in results.boxes:
                    # Clone to avoid in-place update on inference-mode tensors
                    coords = box.xyxy[0].clone()
                    coords[[0, 2]] *= scale_x
                    coords[[1, 3]] *= scale_y

                    x1, y1, x2, y2 = map(int, coords)
                    cls_id = int(box.cls[0])
                    label = model_general.names[cls_id]

                    cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_img, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                return results

            detection_task = asyncio.to_thread(run_yolo)
            depth_task = asyncio.to_thread(compute_depth_map, imgL, imgR)
            results, disparity = await asyncio.gather(detection_task, depth_task)
            results = await results  # nested coroutine

            disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_vis = np.uint8(disp_vis)
            disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

            points_3D = cv2.reprojectImageTo3D(disparity, Q)
            detected_objects = []

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                label = model_general.names[cls_id]

                cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_img, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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

        # === Show updated frames with OpenCV ===
        cv2.imshow("YOLO Detection", annotated_img)
        cv2.imshow("Depth Map", disp_color)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        i += 1
        await asyncio.sleep(0)


# === Main Entrypoint ===
async def main():
    try:
        loop = asyncio.get_running_loop()
        server = ble_server.SafePiBLEServer(loop)
        
        await server.start()
        await capture_and_detect(server)
    except KeyboardInterrupt:
        print("\nInterrupted. Shutting down...")
    finally:
        camera1.stop()
        camera2.stop()
        ser.close()
        cv2.destroyAllWindows()
        print("Cameras and LiDAR stopped.")


if __name__ == "__main__":
    asyncio.run(main())
