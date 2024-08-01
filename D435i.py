from utilites import *

class D435i:
    def __init__(self, image_shape = (640,480), families = None) -> None:
        self.pipeline = start_pipline(image_shape= image_shape)
        profile = self.pipeline.get_active_profile()
        self.detector = detector(families)
        self.color_profile = profile.get_stream(rs.stream.color)
        self.depth_profile = profile.get_stream(rs.stream.depth)
        self.depth_intrinsics = self.color_profile.as_video_stream_profile().get_intrinsics()

        self.K = np.array([[self.depth_intrinsics.fx, 0, self.depth_intrinsics.ppx],[0, self.depth_intrinsics.fy, self.depth_intrinsics.ppy],[0, 0, 1]])
    
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
        real_point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics,[px,py], depth_value)
        real_point = np.array(real_point)
        return real_point
    
    def depth_to_point_cloud(self, depth_image, threshold = None):
        height, width = depth_image.shape
        point_cloud  = []
        for i in range(height):
            for j in range(width):
                point = self.get_3d_coordinates(depth_image, (j,i), threshold=threshold)
                if len(point) > 0:
                    point_cloud.append(self.get_3d_coordinates(depth_image, (j,i)))

        return change_points_to_pcd(np.array(point_cloud))
    
    def get_Transformation_to_tag(self, tag, tag_size = 60, dist_coeffs = np.zeros(5)):

        #计算camera_H_world
        half_size = tag_size / 2
        obj_points = np.array([
        [-half_size, half_size, 0],
        [ half_size, half_size, 0],
        [ half_size,  -half_size, 0],
        [-half_size,  -half_size, 0]
        ], dtype=np.float32)
        img_points = tag.corners
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.K, dist_coeffs)
        if success:
            rvec, tvec = cv2.solvePnPRefineLM(obj_points, img_points, self.K, dist_coeffs, rvec, tvec)
            # 转换旋转向量为旋转矩阵
            R, _ = cv2.Rodrigues(rvec)

            # # 打印外参矩阵
            # print("旋转矩阵 R:")
            # print(R)
            # print("平移向量 t:")
            # print(tvec)
            
            # 组合外参矩阵 [R | t]
            H = np.hstack((R, tvec))
            H = np.concatenate((H, [[0, 0, 0, 1]]), axis = 0)
            # print("外参矩阵 [R | t]:")
            # print(H)
            return H
        else:
            print("solvePnP 求解失败")






