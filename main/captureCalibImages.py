from picamera2 import Picamera2
import time
import cv2
import os

# Create output directories
left_dir = "left"
right_dir = "right"
os.makedirs(left_dir, exist_ok=True)
os.makedirs(right_dir, exist_ok=True)

# Initialize two PiCamera2 instances
camera_left = Picamera2(0)
camera_right = Picamera2(1)

# Configure the cameras
config_left = camera_left.create_still_configuration()
config_right = camera_right.create_still_configuration()

camera_left.configure(config_left)
camera_right.configure(config_right)

# Start the cameras
camera_left.start()
camera_right.start()

time.sleep(2)  # Let the sensors warm up

print("Press ENTER to capture image pairs. Press Ctrl+C to stop.")
capture_count = 0

try:
    while True:
        input("Press ENTER to capture...")

        # Capture images
        img_left = camera_left.capture_array()
        img_right = camera_right.capture_array()

        # Optional: show preview
        cv2.imshow("Left", img_left)
        cv2.imshow("Right", img_right)
        cv2.waitKey(500)

        # Save images
        left_path = os.path.join(left_dir, f"left_{capture_count:02d}.jpg")
        right_path = os.path.join(right_dir, f"right_{capture_count:02d}.jpg")
        cv2.imwrite(left_path, img_left)
        cv2.imwrite(right_path, img_right)

        print(f"Saved pair #{capture_count}:")
        print(f"  {left_path}")
        print(f"  {right_path}")

        capture_count += 1

except KeyboardInterrupt:
    print("\nExiting capture loop.")

finally:
    camera_left.stop()
    camera_right.stop()
    cv2.destroyAllWindows()
