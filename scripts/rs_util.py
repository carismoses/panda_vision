import numpy as np
import pyrealsense2 as rs

def rs_intrinsics_to_opencv_intrinsics(intr):
    D = np.array(intr.coeffs)
    K = np.array([[intr.fx, 0, intr.ppx],
                  [0, intr.fy, intr.ppy],
                  [0, 0, 1]])
    return K, D

def get_serial_number(pipeline_profile):
    return pipeline_profile.get_device().get_info(rs.camera_info.serial_number)

def get_intrinsics(pipeline_profile, stream=rs.stream.color):
    stream_profile = pipeline_profile.get_stream(stream) # Fetch stream profile for depth stream
    intr = stream_profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
    return rs_intrinsics_to_opencv_intrinsics(intr)
