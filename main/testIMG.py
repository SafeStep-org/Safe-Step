import cv2
from ultralytics import YOLO
import numpy as np

# Load YOLO models
model_general = YOLO("yolo11s.pt")      # Replace with your general-purpose YOLO model
model_crosswalk = YOLO("yolov8n.pt")    # Replace with your crosswalk-specific YOLO model

# Load image
image_path = "input.jpg"  # Replace with the path to your image
img_bgr = cv2.imread(image_path)
img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

# Run detection on both models
results_general = model_general(img_rgb)[0]
results_crosswalk = model_crosswalk(img_rgb)[0]

# Combine results
all_results = [(results_general, model_general), (results_crosswalk, model_crosswalk)]

# Draw results
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

# Save the image with annotations
output_path = "output_with_boxes.jpg"
cv2.imwrite(output_path, img_bgr)
print(f"Saved annotated image to {output_path}")
