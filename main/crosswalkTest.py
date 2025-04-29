import cv2
import numpy as np
import os

# Load the ONNX model
net = cv2.dnn.readNetFromONNX("Crosswalks_ONNX_Model.onnx")

# Define input size and scale (adjust based on your training config)
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4

# Class labels (adjust as needed)
class_names = ["crosswalk"]  # Replace with your actual class labels

# Load and process images from a directory
input_dir = "testIMG/images"
output_dir = "crosswalk_output"
os.makedirs(output_dir, exist_ok=True)

def preprocess(image):
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    return blob

def postprocess(image, outputs):
    h, w = image.shape[:2]
    results = []

    # Output parsing depends on your ONNX model's output format
    for det in outputs[0, 0]:  # This assumes a YOLO-like output; adjust as needed
        confidence = float(det[2])
        if confidence > CONFIDENCE_THRESHOLD:
            x1 = int(det[3] * w)
            y1 = int(det[4] * h)
            x2 = int(det[5] * w)
            y2 = int(det[6] * h)
            class_id = int(det[1])
            results.append((x1, y1, x2, y2, confidence, class_id))
    return results

for filename in os.listdir(input_dir):
    if not filename.lower().endswith((".jpg", ".jpeg")):
        continue

    path = os.path.join(input_dir, filename)
    img = cv2.imread(path)

    # Preprocess
    blob = preprocess(img)
    net.setInput(blob)

    # Inference
    outputs = net.forward()

    # Postprocess
    detections = postprocess(img, outputs)

    # Draw results
    for x1, y1, x2, y2, conf, class_id in detections:
        label = f"{class_names[class_id]} {conf:.2f}"
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    out_path = os.path.join(output_dir, filename)
    cv2.imwrite(out_path, img)
    print(f"Saved: {out_path}")
