import cv2
import numpy as np
import os
from pathlib import Path

# === CONFIG ===
MODEL_PATH = "Crosswalks_ONNX_Model.onnx"
INPUT_DIR = "testIMG/images"
OUTPUT_DIR = "crosswalk_output"
INPUT_SIZE = 512
CONF_THRESHOLD = 0.3

# Create output directory if not exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Load ONNX model
net = cv2.dnn.readNetFromONNX(MODEL_PATH)

# Process each JPEG in the input directory
for img_file in Path(INPUT_DIR).glob("*.jpeg"):
    print(f"Processing: {img_file.name}")

    # Read and resize image
    img = cv2.imread(str(img_file))
    img_resized = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
    blob = cv2.dnn.blobFromImage(img_resized, scalefactor=1/255.0, size=(INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False)
    net.setInput(blob)

    # Forward pass
    output = net.forward()

    # Output shape (1, 5, 5376) -> (5376, 5)
    output = output.squeeze().transpose(1, 0)  # Now (5376, 5)

    # Parse detections
    boxes = []
    h_orig, w_orig = img.shape[:2]
    for det in output:
        x, y, w, h, conf = det
        if conf < CONF_THRESHOLD:
            continue

        x1 = int((x - w / 2) * w_orig / INPUT_SIZE)
        y1 = int((y - h / 2) * h_orig / INPUT_SIZE)
        x2 = int((x + w / 2) * w_orig / INPUT_SIZE)
        y2 = int((y + h / 2) * h_orig / INPUT_SIZE)

        boxes.append((x1, y1, x2, y2, conf))

    # Draw detections
    for (x1, y1, x2, y2, conf) in boxes:
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"Crosswalk {conf:.2f}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # Save annotated image
    out_path = Path(OUTPUT_DIR) / img_file.name
    cv2.imwrite(str(out_path), img)

print("✅ Processing complete.")
