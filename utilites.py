import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag
import keyboard
import open3d as o3d

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

def detector(families = None):
    if not families:
        return
    options = apriltag.DetectorOptions(families=families)
    return apriltag.Detector(options)


def get_tags(detector, gray_img):
    
    return detector.detect(gray_img)

def draw_tags_box(tags, frame):

    if len(tags) == 0:
        return frame
    for tag in tags:
        (ptA, ptB, ptC, ptD) = tag.corners
        ptA = (int(ptA[0]), int(ptA[1]))
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))

        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        # 在标签中心绘制圆圈
        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        cv2.putText(frame, str(tag.tag_id), (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return frame

def compute_transformation_matrix_AToB(A_points, B_points):
    # A_points = [sub + [1] for sub in A_points]
    # B_points = [sub + [1] for sub in B_points]
    M = []
    b = []
    for i in range(4):
        M.append([B_points[i][0], B_points[i][1], B_points[i][2], 1, 0, 0, 0, 0, 0, 0, 0, 0])
        M.append([0, 0, 0, 0, B_points[i][0], B_points[i][1], B_points[i][2], 1, 0, 0, 0, 0])
        M.append([0, 0, 0, 0, 0, 0, 0, 0, B_points[i][0], B_points[i][1], B_points[i][2], 1])
        b.append(A_points[i][0])
        b.append(A_points[i][1])
        b.append(A_points[i][2])
    M = np.array(M)
    b = np.array(b)
    if np.linalg.matrix_rank(M) < 12:
        raise np.linalg.LinAlgError("M 矩阵是奇异的或秩不足。")
    h = np.linalg.solve(M, b)
    H = np.array([
        [h[0], h[1], h[2], h[3]],
        [h[4], h[5], h[6], h[7]],
        [h[8], h[9], h[10], h[11]],
        [0, 0, 0, 1]
    ])
    return H  

def wait_for_c_key():
    print("Press 'c' to continue...")
    while True:
        if keyboard.is_pressed('c'):
            print("'c' pressed. Continuing...")
            break          

def change_points_to_pcd(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd








