import serial
import time
import torch
import cv2
from picamera2 import Picamera2
from libcamera import controls
from ultralytics import YOLO

# Initialize Cameras
print("Initializing cameras...")
camera1 = Picamera2(0)  # First camera (wide-angle or primary)
camera2 = Picamera2(1)  # Second camera (secondary)

camera1.set_controls({"AfMode": controls.AfModeEnum.Continuous})
camera2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# Start cameras
camera1.start(show_preview=True)
camera2.start(show_preview=True)
print("Cameras initialized and started.")

# Initialize TF-Luna LiDAR
print("Initializing TF-Luna LiDAR...")
ser = serial.Serial("/dev/ttyAMA0", 115200)

# Load YOLOv5 Model
print("Loading YOLO11s model...")
model = YOLO("yolo11s.pt")

def read_tfluna_data():
    """Reads distance, signal strength, and temperature from the TF-Luna."""
    time.sleep(1)  # Sleep 1000ms
    while True:
        counter = ser.in_waiting  # Check available bytes in the buffer
        if counter > 8:  # Ensure at least one full frame is available
            bytes_serial = ser.read(9)  # Read exactly 9 bytes
            ser.reset_input_buffer()  # Clear buffer to avoid stale data

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:  # Frame header validation (Python 3 portion)
                distance = bytes_serial[2] + bytes_serial[3] * 256  # Distance in cm
                strength = bytes_serial[4] + bytes_serial[5] * 256  # Signal strength
                temperature = bytes_serial[6] + bytes_serial[7] * 256  # Temperature raw data
                temperature = (temperature / 8.0) - 256.0  # Convert to Celsius
                print("TF-Luna python3 portion")
                print(f"Distance: {distance} cm")
                print(f"Strength: {strength}")
                if temperature != 0:
                    print(f"Chip Temperature: {temperature:.1f} °C")

            elif bytes_serial[0] == ord("Y") and bytes_serial[1] == ord("Y"):  # Frame header validation (Python 2 portion)
                distL = int(bytes_serial[2].hex(), 16)
                distH = int(bytes_serial[3].hex(), 16)
                stL = int(bytes_serial[4].hex(), 16)
                stH = int(bytes_serial[5].hex(), 16)
                distance = distL + distH * 256
                strength = stL + stH * 256
                tempL = int(bytes_serial[6].hex(), 16)
                tempH = int(bytes_serial[7].hex(), 16)
                temperature = tempL + tempH * 256
                temperature = (temperature / 8) - 256
                print("TF-Luna python2 portion")
                print(f"Distance: {distance} cm")
                print(f"Strength: {strength}")
                print(f"Chip Temperature: {temperature:.1f} °C")
            ser.reset_input_buffer()

# Main loop to capture data from cameras and LiDAR
try:
    while True:
        # Capture images from both cameras
        print("Capturing images...")
        img1 = camera1.capture_array()  # Capture image from Camera 1
        img2 = camera2.capture_array()  # Capture image from Camera 2

        # Convert images to proper format for YOLOv5
        img1_rgb = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
        img2_rgb = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

        # Run YOLOv5 inference on both images
        print("Running object detection...")
        results1 = model(img1_rgb)  # Inference on Camera 1 image
        # results2 = model(img2_rgb)  # Inference on Camera 2 image

        # Print detected objects
        print("Camera 1 Results:")
        results1.print()  # Prints detected labels, confidence scores, etc.
        
        print("Camera 2 Results:")
        # results2.print()

        # Visualize results (optional)
        results1.show()  # Displays image with bounding boxes
        # results2.show()

        # Read data from TF-Luna LiDAR
        print("Reading LiDAR data...")
        read_tfluna_data()

        time.sleep(0.5)  # Adjust delay as needed for your application

except KeyboardInterrupt:
    print("Stopping program...")

finally:
    # Stop cameras and close serial port
    camera1.stop()
    camera2.stop()
    ser.close()
    print("Cameras and LiDAR stopped.")
