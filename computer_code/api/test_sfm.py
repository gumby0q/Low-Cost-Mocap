
# import cv2

# # Load images
# img1 = cv2.imread('image1.jpg')
# img2 = cv2.imread('image2.jpg')

# # Create a list of images
# images = [img1, img2]

# # Create a Structure From Motion (SFM) object
# sfm = cv2.SFM_create()

# # # Perform SFM reconstruction
# # reconstruction, points3d = sfm.reconstruct(images)

# # # Print the reconstructed 3D points
# # print("Reconstructed 3D points:")
# # for point in points3d:
# #     print(point)

# # Visualize the reconstructed scene (optional)
# # You may need additional visualization code depending on your requirements
# # For example, you could use matplotlib to plot the reconstructed 3D points

# # Display images with key points (optional)
# # You can use OpenCV's drawing functions to overlay key points on the images

# # Display the result (optional)
# # You can use OpenCV's imshow function to display the reconstructed scene or any other visualizations

# --------------------------------------------------------------------
# --------------------------------------------------------------------
# --------------------------------------------------------------------


# import matplotlib.pyplot as plt

# from mpl_toolkits.mplot3d import axes3d

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')

# # Grab some example data and plot a basic wireframe.
# X, Y, Z = axes3d.get_test_data(0.05)
# ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10)

# # Set the axis labels
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')

# # Rotate the axes and update
# for angle in range(0, 360*4 + 1):
#     # Normalize the angle to the range [-180, 180] for display
#     angle_norm = (angle + 180) % 360 - 180

#     # Cycle through a full rotation of elevation, then azimuth, roll, and all
#     elev = azim = roll = 0
#     if angle <= 360:
#         elev = angle_norm
#     elif angle <= 360*2:
#         azim = angle_norm
#     elif angle <= 360*3:
#         roll = angle_norm
#     else:
#         elev = azim = roll = angle_norm

#     # Update the axis view and title
#     ax.view_init(elev, azim, roll)
#     plt.title('Elevation: %d°, Azimuth: %d°, Roll: %d°' % (elev, azim, roll))

#     plt.draw()
#     plt.pause(.001)


# --------------------------------------------------------------------
# --------------------------------------------------------------------
# --------------------------------------------------------------------

# import time

# import matplotlib.pyplot as plt
# import numpy as np

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')

# # Make the X, Y meshgrid.
# xs = np.linspace(-1, 1, 50)
# ys = np.linspace(-1, 1, 50)
# X, Y = np.meshgrid(xs, ys)

# # Set the z axis limits, so they aren't recalculated each frame.
# ax.set_zlim(-1, 1)

# # Begin plotting.
# wframe = None
# tstart = time.time()
# for phi in np.linspace(0, 180. / np.pi, 100):
#     # If a line collection is already remove it before drawing.
#     if wframe:
#         wframe.remove()
#     # Generate data.
#     Z = np.cos(2 * np.pi * X + phi) * (1 - np.hypot(X, Y))
#     # Plot the new wireframe and pause briefly before continuing.
#     wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2)
#     plt.pause(.001)

# print('Average FPS: %f' % (100 / (time.time() - tstart)))


# --------------------------------------------------------------------
# --------------------------------------------------------------------
# --------------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np

np.random.seed(19680801)
data = np.random.random((50, 50, 50))

fig, ax = plt.subplots()

for i, img in enumerate(data):
    ax.clear()
    ax.imshow(img)
    ax.set_title(f"frame {i}")
    # Note that using time.sleep does *not* work here!
    plt.pause(0.1)


