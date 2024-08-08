import sys
import os

sys.path.append(os.path.abspath('../'))
from Point_Cloud_Registration.utilities import  *
import pyrealsense2 as rs
import numpy as np
import cv2
from utilites import *
import apriltag
import matplotlib.pyplot as plt
from D435i import D435i


if __name__ == "__main__":
    myD435i = D435i(families='tag36h11')
    pcd_cloud = []


    while True:
        depth_image, color_image = myD435i.get_image()
        gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = myD435i.get_tags(gray_img)
        color_image = draw_tags_box(tags, color_image)
        # for tag in tags:
        #     real_center = myD435i.get_3d_coordinates(depth_image, tag.center)
        #     # print(real_center)
        #     H = myD435i.get_Transformation_to_tag(tag)
        #     # h = np.zeros((4,4))
        #     # real_center = myD435i.get_3d_coordinates(depth_image, tag.center)
        #     # h[0:3,0:3] = tag.homography
        #     # print(h)
        
            

        cv2.imshow('RealSense Color', color_image)
        keyvalue = cv2.waitKey(1) & 0xFF
        #当按下c时截取图片并将其转换成3d点云
        #按下q退出
        if keyvalue == ord('q'):
            break
        elif keyvalue == ord('c'):
            if len(tags) > 0:
                H = myD435i.get_Transformation_to_tag(tags[0],tag_size=70)
                pcd = myD435i.depth_to_point_cloud(depth_image, 1000)
                pcd.transform(np.linalg.inv(H))
                pcd_cloud.append(pcd)
                print('Done')
            else:
                print('No tag deteced')
    myD435i.stop_pipline()
    cv2.destroyAllWindows()
    for i, pcd in enumerate(pcd_cloud):
        filename = f"output/point_cloud_{i+1}.ply"
        o3d.io.write_point_cloud(filename, pcd)
        print(f"点云数据已保存到 {filename}")

