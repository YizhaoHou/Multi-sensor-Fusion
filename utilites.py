import pyrealsense2 as rs
import numpy as np
import cv2

def start_pipline(image_shape = (640,480)):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, image_shape[0], image_shape[1], rs.format.z16, 30)
    config.enable_stream(rs.stream.color, image_shape[0], image_shape[1], rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline
def get_image(pipeline):
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        print("No image readed")
        return
    else:
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
    return depth_image, color_image