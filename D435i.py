from utilites import *
class D435i:
    def __init__(self, image_shape = (640,480), families = None) -> None:
        self.pipline = start_pipline(image_shape= image_shape)
        profile = self.pipeline.get_active_profile()
        self.detector = detector(families)
        self.color_profile = profile.get_stream(rs.stream.color)
        self.depth_profile = profile.get_stream(rs.stream.depth)
    
    def get_image(self):
        return get_image(self.pipline)
    
    def get_tags(self, gray_img):
        return get_tags(self.detector, gray_img)


    def stop_pipline(self):
        self.pipline.stop()
