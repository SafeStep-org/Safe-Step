import cv2
import numpy as np
import glob
import os

# Load the ONNX model
net = cv2.dnn.readNetFromONNX("Crosswalks_ONNX_Model.onnx")

# Set input dimensions expected by the model
INPUT_WIDTH = 512
INPUT_HEIGHT = 512
CONFIDENCE_THRESHOLD = 0.5

# Directory with JPEG images
image_dir = "testIMG/images"
output_dir = "crosswalk_output"
os.makedirs(output_dir, exist_ok=True)

# Process all JPEG images
for image_path in glob.glob(os.path.join(image_dir, "*.jpeg")):
    print(f"Processing {image_path}...")

    image = cv2.imread(image_path)
    if image is None:
        print(f"Failed to read image {image_path}")
        continue

    orig_h, orig_w = image.shape[:2]

    # Preprocess
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
    net.setInput(blob)

    # Run inference
    outputs = net.forward()

    # You may need to adjust how predictions are parsed depending on model output format
    # Here we assume output is [num_detections, 6] = [x, y, w, h, conf, class]
    for detection in outputs[0]:
        x, y, w, h, conf, cls = detection
        if conf < CONFIDENCE_THRESHOLD:
            continue

        # Scale coordinates back to original image size
        x = int((x - w / 2) * orig_w)
        y = int((y - h / 2) * orig_h)
        w = int(w * orig_w)
        h = int(h * orig_h)

        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(image, f"{int(cls)} {conf:.2f}", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Save result
    filename = os.path.basename(image_path)
    output_path = os.path.join(output_dir, filename)
    cv2.imwrite(output_path, image)
    print(f"Saved annotated image to {output_path}")
