from utilites import *
class D435i:
    def __init__(self, image_shape = (640,480), families = None) -> None:
        self.pipeline = start_pipline(image_shape= image_shape)
        profile = self.pipeline.get_active_profile()
        self.detector = detector(families)
        self.color_profile = profile.get_stream(rs.stream.color)
        self.depth_profile = profile.get_stream(rs.stream.depth)
    
    def get_image(self):
        return get_image(self.pipeline)
    
    def get_tags(self, gray_img):
        return get_tags(self.detector, gray_img)


    def stop_pipline(self):
        self.pipeline.stop()

    
    def get_3d_coordinates(self, depth_frame, point):
        if not depth_frame:
            return None
        px, py = point
        px = np.round(px)
        py = np.round(py)
        depth_value = depth_frame.get_distance(px,py)
        if depth_value == 0:
            return None
        real_point = rs.rs2_deproject_pixel_to_point(self.depth_profile.as_video_stream_profile().get_intrinsics(),[px,py], depth_value)
        return real_point
