from picamera2 import Picamera2, Preview
import time
import cv2
import os

# Create output directories
left_dir = "left"
right_dir = "right"
os.makedirs(left_dir, exist_ok=True)
os.makedirs(right_dir, exist_ok=True)

# Initialize PiCamera2 instances for stereo cameras
camera_left = Picamera2(0)
camera_right = Picamera2(1)

# Configure both cameras
config_left = camera_left.create_preview_configuration()
config_right = camera_right.create_preview_configuration()

camera_left.configure(config_left)
camera_right.configure(config_right)

camera_left.start_preview(Preview.QTGL)
camera_left.start()
camera_right.start()

time.sleep(2)  # Give sensors time to stabilize

print("Press enter to capture image pairs.")
capture_count = 0

try:
    while True:
        # Capture a preview frame from the left camera
        img_left = camera_left.capture_array()

        

        input()
        # Capture full-resolution images from both cameras
        img_left_full = camera_left.capture_array()
        img_right_full = camera_right.capture_array()

        # Save images
        left_path = os.path.join(left_dir, f"left_{capture_count:02d}.jpg")
        right_path = os.path.join(right_dir, f"right_{capture_count:02d}.jpg")
        cv2.imwrite(left_path, img_left_full)
        cv2.imwrite(right_path, img_right_full)
        print(f"Saved image pair #{capture_count}: {left_path}, {right_path}")
        capture_count += 1

finally:
    camera_left.stop()
    camera_right.stop()
