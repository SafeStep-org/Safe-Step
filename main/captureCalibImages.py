import cv2
import os

# Output directories
left_dir = 'left'
right_dir = 'right'
os.makedirs(left_dir, exist_ok=True)
os.makedirs(right_dir, exist_ok=True)

# Open both cameras
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)

if not cap_left.isOpened() or not cap_right.isOpened():
    print("Error: One or both cameras could not be opened.")
    exit()

print("Press SPACE to capture image pairs, ESC to exit.")
capture_count = 0

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not ret_left or not ret_right:
        print("Failed to grab frames.")
        break

    # Resize windows for viewing convenience (optional)
    display_left = cv2.resize(frame_left, (640, 480))
    display_right = cv2.resize(frame_right, (640, 480))

    cv2.imshow('Left Camera', display_left)
    cv2.imshow('Right Camera', display_right)

    key = cv2.waitKey(1)
    if key == 27:  # ESC key
        break
    elif key == 32:  # SPACE key
        left_path = os.path.join(left_dir, f"left_{capture_count:02d}.jpg")
        right_path = os.path.join(right_dir, f"right_{capture_count:02d}.jpg")

        cv2.imwrite(left_path, frame_left)
        cv2.imwrite(right_path, frame_right)
        print(f"Saved: {left_path} and {right_path}")
        capture_count += 1

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
