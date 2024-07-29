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

    
    def get_3d_coordinates(self, depth_frame, point, threshold = None):
        px, py = point
        px = int(np.round(px))
        py = int(np.round(py))
        depth_value = depth_frame[py, px]
        if depth_value == 0:
            return []
        if threshold:
            if depth_value > threshold:
                return []
        real_point = rs.rs2_deproject_pixel_to_point(self.depth_profile.as_video_stream_profile().get_intrinsics(),[px,py], depth_value)
        real_point = np.array(real_point)/1000
        return real_point
    
    def depth_to_point_cloud(self, depth_image, threshold = None):
        height, width = depth_image.shape
        point_cloud  = []
        for i in range(height):
            for j in range(width):
                point = self.get_3d_coordinates(depth_image, (j,i), threshold=threshold)
                if len(point) > 0:
                    point_cloud.append(self.get_3d_coordinates(depth_image, (j,i)))

        return np.array(point_cloud)


