#!/usr/bin/env python
 
import cv2
import numpy as np
import os
import glob
import json
 
cam_images_folder_name = 'cam_1'
cam_images_folder_name_calibrated = f'{cam_images_folder_name}_c'
# cam_images_folder_name = 'cam_1'

# Defining the dimensions of checkerboard
# CHECKERBOARD = (6,9)
CHECKERBOARD = (5,6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 
 
 
# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None
 
# Extracting path of individual image stored in a given directory
images = glob.glob(f'./{cam_images_folder_name}/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    # ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
     
    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display 
    them on the images of checker board
    """
    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
         
        imgpoints.append(corners2)
 
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
     
    # cv2.imshow('img',img)
    # cv2.waitKey(0)
    
    new_frame_name = cam_images_folder_name_calibrated + '/' + os.path.basename(fname)
    # print(new_frame_name)
    cv2.imwrite(new_frame_name, img)

 
# cv2.destroyAllWindows()
 
h,w = img.shape[:2]
 
"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
 
print("Camera matrix : \n")
# print(mtx)
print(json.dumps(mtx.tolist()))
print("dist : \n")
# print(dist)
print(json.dumps(dist.tolist()))
# print("rvecs : \n")
# print(rvecs)
# print("tvecs : \n")
# print(tvecs)



# -----

# cam 0

# Camera matrix : 

# [[293.37352356   0.         148.83826395]
#  [  0.         293.62691058 111.79537397]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.45485019  0.2550664   0.00066252  0.00063356 -0.09096037]]

# Camera matrix : 

# [[293.27414483   0.         147.12726117]
#  [  0.         293.30479926 110.98823837]
#  [  0.           0.           1.        ]]
# dist : 

# [[-4.47296210e-01  2.18968662e-01 -1.38218013e-04  1.25328277e-03
#   -4.54514945e-02]]

# Camera matrix : 

# [[293.72298031   0.         149.37364445]
#  [  0.         293.91614053 110.97211794]
#  [  0.           0.           1.        ]]
# dist : 

# [[-4.51543547e-01  2.30999426e-01 -3.70807888e-04  1.38167804e-04
#   -2.23855337e-02]]


# cam 1

# Camera matrix : 

# [[296.90923342   0.         160.59761054]
#  [  0.         297.1554197  110.21182143]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.44448608  0.16638235 -0.00134475  0.00131194  0.22851977]]

# Camera matrix : 

# [[296.95068856   0.         160.5867128 ]
#  [  0.         297.7072183  109.85623029]
#  [  0.           0.           1.        ]]
# dist : 

# [[-4.66763298e-01  3.50056767e-01 -3.04871017e-04  1.58960039e-03
#   -1.88932634e-01]]

# Camera matrix : 

# [[298.04869792   0.         156.56849437]
#  [  0.         298.63108318 111.31279276]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.44821471  0.23605039 -0.00140101  0.00335551 -0.05762157]]


# cam 2

# Camera matrix : 

# [[292.89988147   0.         162.16429663]
#  [  0.         293.18181331 112.18103778]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.46861154  0.29509876 -0.00055081  0.00161669 -0.09578225]]

# Camera matrix : 

# [[293.78759057   0.         156.26980361]
#  [  0.         294.14708633 114.97939663]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.48164726  0.3738794  -0.0018836   0.00466947 -0.22736521]]

# Camera matrix : 

# [[294.30331913   0.         158.44340207]
#  [  0.         294.7022009  113.92254607]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.46220657  0.26541399 -0.00172333  0.00328379 -0.06275199]]


# cam 3

# [[292.89998644   0.         149.80186972]
#  [  0.         293.21216775 113.91481391]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.45435649  0.2203447  -0.00133407  0.00378088  0.01993536]]

# Camera matrix : 

# [[294.6740507    0.         149.56935126]
#  [  0.         295.02730928 114.67540133]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.47294806  0.35054603 -0.00149444  0.00356464 -0.18970455]]

# Camera matrix : 

# [[293.24714118   0.         149.69690819]
#  [  0.         293.38158754 116.53691307]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.45061978  0.19828234 -0.00243227  0.00301325  0.01950912]]

# higher resolution

# [[586.09466821   0.         286.51962831]
#  [  0.         586.71540335 231.12671591]
#  [  0.           0.           1.        ]]
# dist : 

# [[-0.47535581  0.32905769 -0.00195123  0.00516486 -0.14405701]]

# Camera matrix : 

# [[586.44899095   0.         291.30989143]
#  [  0.         587.97005441 225.24612814]
#  [  0.           0.           1.        ]]
# dist : 

# [[-4.85644035e-01  4.09259595e-01  2.30160362e-04  4.83862842e-03
#   -2.57657774e-01]]