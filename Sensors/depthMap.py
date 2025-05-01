import cv2
import numpy as np
import matplotlib.pyplot as plt
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
    numDisparities=16*8,  # more disparities
    blockSize=3,          # smaller blocks
    P1=8 * 3 * 3**2,
    P2=32 * 3 * 3**2,
    disp12MaxDiff=1,      # left-right check
    uniquenessRatio=10,   # better matching
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)


print("Starting live depth map... (Press CTRL+C to stop)")

# Setup matplotlib interactive plot
plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
im = ax.imshow(np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8))
plt.axis('off')

try:
    while True:
        imgL = camL.capture_array()
        imgR = camR.capture_array()

        rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
        rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)

        grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

        disparity = stereo.compute(grayL, grayR).astype(np.float32) / 16.0

        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = np.uint8(disp_vis)

        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        # Update matplotlib plot
        im.set_data(cv2.cvtColor(disp_color, cv2.COLOR_BGR2RGB))
        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    camL.stop()
    camR.stop()
    plt.close()
