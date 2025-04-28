import cv2
import numpy as np
import glob

# Calibration settings
board_size = (10, 7)  # (width, height) of inner checkerboard corners
square_size = 0.016  # Real size of one square (in meters)

# Prepare object points
objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store points
objpoints = []  # 3D points
imgpointsL = [] # 2D points for left
imgpointsR = [] # 2D points for right

images_left = sorted(glob.glob("left/*.png"))
images_right = sorted(glob.glob("right/*.png"))

for imgL_path, imgR_path in zip(images_left, images_right):
    imgL = cv2.imread(imgL_path)
    imgR = cv2.imread(imgR_path)

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, board_size, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, board_size, None)

    if retL and retR:
        objpoints.append(objp)
        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)

# Calibrate individual cameras
retL, mtxL, distL, _, _ = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
retR, mtxR, distR, _, _ = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)

# Stereo calibration
flags = cv2.CALIB_FIX_INTRINSIC
criteria = (cv2.TermCriteria_MAX_ITER + cv2.TermCriteria_EPS, 100, 1e-5)

retStereo, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR,
    mtxL, distL,
    mtxR, distR,
    grayL.shape[::-1],
    criteria=criteria,
    flags=flags
)

# Save calibration results
np.savez("stereo_calib_data.npz", 
         mtxL=mtxL, distL=distL, 
         mtxR=mtxR, distR=distR, 
         R=R, T=T, E=E, F=F)

print("Stereo calibration completed and saved to 'stereo_calib_data.npz'.")
