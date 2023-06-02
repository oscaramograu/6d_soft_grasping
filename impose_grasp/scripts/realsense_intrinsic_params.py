import pyrealsense2 as rs
import numpy as np
import os

# Create a pipeline
pipeline = rs.pipeline()

# Start streaming with default configuration
pipeline.start()

# Get the first connected device
device = pipeline.get_active_profile().get_device()

# Get the depth sensor
depth_sensor = device.query_sensors()[0]

# Get the depth stream profile
depth_stream = next(p for p in depth_sensor.profiles if p.stream_type() == rs.stream.depth)

# Get the intrinsic parameters
intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

# Create the intrinsic matrix
intrinsic_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                             [0, intrinsics.fy, intrinsics.ppy],
                             [0, 0, 1]])

# Save the intrinsic matrix to a text file
save_path = os.path.join(os.getcwd(), "data", "calibration")
np.savetxt(os.path.join(save_path, 'intrinsic_matrix.txt'), intrinsic_matrix)


# Print the intrinsic matrix
print("Intrinsic Matrix:")
print(intrinsic_matrix)

# Stop the pipeline
pipeline.stop()
