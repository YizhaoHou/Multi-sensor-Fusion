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