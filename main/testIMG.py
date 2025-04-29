import cv2
import os
from ultralytics import YOLO
import numpy as np

# Load YOLO models
model_general = YOLO("yolo11s.pt")      # General-purpose model
model_crosswalk = YOLO("yolov8n.pt")    # Crosswalk-specific model

# Directory with input images
input_dir = "testIMG/images"  # Change this to your folder path
output_dir = "output"
os.makedirs(output_dir, exist_ok=True)

# Supported image extensions
image_extensions = (".jpg", ".jpeg")

# Process each image in the directory
for filename in os.listdir(input_dir):
    if not filename.lower().endswith(image_extensions):
        continue

    input_path = os.path.join(input_dir, filename)
    output_path = os.path.join(output_dir, filename.rsplit(".", 1)[0] + "_boxed.jpg")

    print(f"Processing {filename}...")

    # Load and convert image
    img_bgr = cv2.imread(input_path)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    # Run detection
    results_general = model_general(img_rgb)[0]
    results_crosswalk = model_crosswalk(img_rgb)[0]
    all_results = [(results_general, model_general), (results_crosswalk, model_crosswalk)]

    # Draw boxes
    for results, model in all_results:
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = box.conf[0].item()

            # Draw bounding box and label
            cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label_text = f"{label} {conf:.2f}"
            cv2.putText(img_bgr, label_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Save annotated image
    cv2.imwrite(output_path, img_bgr)
    print(f"Saved to {output_path}")
