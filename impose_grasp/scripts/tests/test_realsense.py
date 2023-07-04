import cv2
import numpy as np
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.models.cameras.realsense_D415 import D415


# Start streaming
cam = D415(name="realsense_D415")
cam.start()

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = cam.grab_frame()
        color_frame = frames.rgb
        depth = frames.depth

        # Display the resulting frame
        # cv2.imshow('RealSense Stream', color_frame)
        cv2.imshow('RealSense Stream', depth)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    cam.stop()

# Close all OpenCV windows
cv2.destroyAllWindows()
