import cv2
import numpy as np
from picamera2 import Picamera2

# Load calibration
calib_data = np.load('stereo_calib_data.npz')
mtxL, distL = calib_data['mtxL'], calib_data['distL']
mtxR, distR = calib_data['mtxR'], calib_data['distR']
R, T = calib_data['R'], calib_data['T']

# Initialize cameras
camL = Picamera2(0)
camR = Picamera2(1)
camL.start()
camR.start()

# Get image size
frameL = camL.capture_array()
frameR = camR.capture_array()
img_size = (frameL.shape[1], frameL.shape[0])

# Stereo rectify
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, img_size, R, T)

# Precompute rectification maps
mapLx, mapLy = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, img_size, cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, img_size, cv2.CV_32FC1)

# StereoSGBM matcher
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16*5,   # must be divisible by 16
    blockSize=5,
    P1=8 * 3 * 5 ** 2,
    P2=32 * 3 * 5 ** 2,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

print("Starting live depth map...")

while True:
    imgL = camL.capture_array()
    imgR = camR.capture_array()

    # Rectify
    rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)

    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

    # Compute disparity
    disparity = stereo.compute(grayL, grayR).astype(np.float32) / 16.0

    # Normalize disparity for visualization
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)

    # Color map for better visualization
    disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

    cv2.imshow("Disparity", disp_color)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
camL.stop()
camR.stop()
