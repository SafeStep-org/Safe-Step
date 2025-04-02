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
camera2 = Picamera2()  # Second camera (secondary)

camera1.set_controls({"AfMode": controls.AfModeEnum.Continuous})
camera2.set_controls({"AfMode": controls.AfModeEnum.Continuous})

# Start cameras
camera1.start(show_preview=True)
camera2.start(show_preview=True)
print("Cameras initialized and started.")

# Initialize TF-Luna LiDAR
print("Initializing TF-Luna LiDAR...")
ser = serial.Serial("/dev/serial0", 115200, timeout=0)  # UART port for LiDAR

# Load YOLOv5 Model
print("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Use 'yolov5n' for Nano or 'yolov5x' for larger weights

def read_tfluna_data():
    """Reads distance, signal strength, and temperature from the TF-Luna."""
    while True:
        counter = ser.in_waiting  # Check available bytes in the buffer
        if counter > 8:  # Ensure at least one full frame is available
            bytes_serial = ser.read(9)  # Read exactly 9 bytes
            ser.reset_input_buffer()  # Clear buffer to avoid stale data

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:  # Frame header validation
                distance = bytes_serial[2] + bytes_serial[3] * 256  # Distance in cm
                strength = bytes_serial[4] + bytes_serial[5] * 256  # Signal strength
                temperature = bytes_serial[6] + bytes_serial[7] * 256  # Temperature raw data
                temperature = (temperature / 8.0) - 256.0  # Convert to Celsius
                return distance / 100.0, strength, temperature

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
        results2 = model(img2_rgb)  # Inference on Camera 2 image

        # Print detected objects
        print("Camera 1 Results:")
        results1.print()  # Prints detected labels, confidence scores, etc.
        
        print("Camera 2 Results:")
        results2.print()

        # Visualize results (optional)
        results1.show()  # Displays image with bounding boxes
        results2.show()

        # Read data from TF-Luna LiDAR
        print("Reading LiDAR data...")
        distance, strength, temperature = read_tfluna_data()

        print(f"LiDAR - Distance: {distance:.2f} m, Strength: {strength}, Temperature: {temperature:.1f} °C")

        time.sleep(0.5)  # Adjust delay as needed for your application

except KeyboardInterrupt:
    print("Stopping program...")

finally:
    # Stop cameras and close serial port
    camera1.stop()
    camera2.stop()
    ser.close()
    print("Cameras and LiDAR stopped.")
