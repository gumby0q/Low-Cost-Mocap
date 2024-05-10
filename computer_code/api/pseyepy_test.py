



# from pseyepy import Camera

# # initialize all connected cameras
# print("Camera.RES_SMALL", Camera.RES_SMALL)
# c = Camera()
# # c = Camera(fps=90, resolution=Camera.RES_SMALL, gain=10, exposure=100)

# # read from the camera/s
# frame, timestamp = c.read()

# # when finished, close the camera
# c.end()



from pseyepy import Camera, Stream

c = cameras = Camera(fps=90, resolution=Camera.RES_SMALL)
# c = cameras = Camera(fps=90, resolution=Camera.RES_SMALL, gain=10, exposure=100)
# c = Camera(fps=30, colour=[False,True], gain=50, vflip=[True, False])
# s = Stream(c, file_name='example_movie.avi', display=True)

# c = Camera() # initialize a camera
# s = Stream(c, file_name='example_movie.avi', display=True, codec='png') # begin saving data to files
s = Stream(c, file_name='example_movie.avi', display=True) # begin saving data to files

# read from the camera/s
# frame, timestamp = c.read()

# print("Camera.RES_SMALL", Camera.RES_SMALL)
# print("frame", frame)
# print("Camera.RES_SMALL", Camera.RES_SMALL)


# when finished, close the stream
s.end()
c.end()


# from pseyepy import Camera, Display
# import cv2  # OpenCV for image handling
# import time

# cam_index = 1

# # Initialize camera
# # cams = Camera(fps=90, resolution=Camera.RES_SMALL, colour=True, gain=10, exposure=100)
# cams = Camera(fps=90, resolution=Camera.RES_SMALL, gain=10, exposure=100)

# for i in range(20):  # Capture 10 images
#     frame, timestamp = cams.read()
#     filename = f'cam_{cam_index}/image_{i}.jpg'
#     cv2.imwrite(filename, frame)
#     time.sleep(1)  # Wait a second between captures

# # # Grab a frame
# # frame, timestamp = cams.read()
# # # Save the frame as a JPEG file
# # cv2.imwrite('image2.jpg', frame)

# # Clean up
# cams.end()
