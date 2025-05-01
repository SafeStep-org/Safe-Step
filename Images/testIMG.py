import cv2
import os
import numpy as np
from pathlib import Path
from ultralytics import YOLO

# === CONFIGURATION ===
INPUT_DIR = "testIMG/images"
OUTPUT_DIR = "combined_output"
YOLO_MODEL_PATH = "yolo11s.pt"               # General-purpose YOLO model
CROSSWALK_MODEL_PATH = "Crosswalks_ONNX_Model.onnx"  # ONNX crosswalk model
INPUT_SIZE = 512
CONF_THRESHOLD_ONNX = 0.3
os.makedirs(OUTPUT_DIR, exist_ok=True)

# === LOAD MODELS ===
model_general = YOLO(YOLO_MODEL_PATH)
net_crosswalk = cv2.dnn.readNetFromONNX(CROSSWALK_MODEL_PATH)

# === PROCESS EACH IMAGE ===
image_extensions = (".jpg", ".jpeg")
for img_path in Path(INPUT_DIR).glob("*.jp*g"):
    print(f"Processing: {img_path.name}")
    img_bgr = cv2.imread(str(img_path))
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    h_orig, w_orig = img_bgr.shape[:2]

    # ==== YOLO INFERENCE ====
    results_general = model_general(img_rgb)[0]

    for box in results_general.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls_id = int(box.cls[0])
        label = model_general.names[cls_id]
        conf = box.conf[0].item()

        # Draw general object bounding box
        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img_bgr, f"{label} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # ==== ONNX CROSSWALK INFERENCE ====
    img_resized = cv2.resize(img_bgr, (INPUT_SIZE, INPUT_SIZE))
    blob = cv2.dnn.blobFromImage(img_resized, scalefactor=1/255.0, size=(INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False)
    net_crosswalk.setInput(blob)
    output = net_crosswalk.forward()
    output = output.squeeze().transpose(1, 0)  # Shape: (5376, 5)

    for det in output:
        x, y, w, h, conf = det
        if conf < CONF_THRESHOLD_ONNX:
            continue

        x1 = int((x - w / 2) * w_orig / INPUT_SIZE)
        y1 = int((y - h / 2) * h_orig / INPUT_SIZE)
        x2 = int((x + w / 2) * w_orig / INPUT_SIZE)
        y2 = int((y + h / 2) * h_orig / INPUT_SIZE)

        # Draw crosswalk bounding box
        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(img_bgr, f"Crosswalk {conf:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # ==== SAVE RESULT ====
    out_path = Path(OUTPUT_DIR) / img_path.name
    cv2.imwrite(str(out_path), img_bgr)
    print(f"Saved: {out_path}")

print("✅ All images processed.")
