import cv2
import os
from ultralytics import YOLO

# Load the YOLOv8n model (crosswalk or other specialized use)
model = YOLO("yolov8n.pt")

# Input and output directories
input_dir = "testIMG/images"   # Replace with your folder containing JPEGs
output_dir = "crosswalk_output"
os.makedirs(output_dir, exist_ok=True)

# Supported image extensions
image_extensions = (".jpg", ".jpeg")

# Process each image
for filename in os.listdir(input_dir):
    if not filename.lower().endswith(image_extensions):
        continue

    input_path = os.path.join(input_dir, filename)
    output_path = os.path.join(output_dir, filename.rsplit(".", 1)[0] + "_boxed.jpg")

    print(f"Processing {filename}...")

    # Load and convert image
    img_bgr = cv2.imread(input_path)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    # Run detection with YOLOv8n
    results = model(img_rgb)[0]

    # Draw boxes
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls_id = int(box.cls[0])
        label = model.names[cls_id]
        conf = box.conf[0].item()

        # Draw bounding box and label
        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
        label_text = f"{label} {conf:.2f}"
        cv2.putText(img_bgr, label_text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Save the result
    cv2.imwrite(output_path, img_bgr)
    print(f"Saved to {output_path}")
