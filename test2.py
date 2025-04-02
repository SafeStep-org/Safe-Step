# Import required libraries
from picamera2 import Picamera2
from libcamera import controls
import serial
import time
import torch
import cv2

# Initialize Cameras
print("Initializing cameras...")
camera1 = Picamera2()  # First camera (wide-angle or primary)

camera1.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# Start cameras
camera1.start(show_preview=True)
print("Cameras initialized and started.")

# Load YOLOv5 Model
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Use 'yolov5n' for Nano or 'yolov5x' for larger weights

# Main loop to capture data from cameras and LiDAR
try:
    while True:
        # Capture images from both cameras
        print("Capturing images...")
        img1 = camera1.capture_array()  # Capture image from Camera 1

        # Convert images to proper format for YOLOv5
        img1_rgb = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)

        # Run YOLOv5 inference on both images
        print("Running object detection...")
        results1 = model(img1_rgb)  # Inference on Camera 1 image

        # Print detected objects
        print("Camera 1 Results:")
        results1.print()  # Prints detected labels, confidence scores, etc.

        # Visualize results (optional)
        results1.show()  # Displays image with bounding boxes

        time.sleep(0.5)  # Adjust delay as needed for your application

except KeyboardInterrupt:
    print("Stopping program...")

finally:
    # Stop cameras and close serial port
    camera1.stop()
    print("Cameras and LiDAR stopped.")
