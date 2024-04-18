import pyrealsense2 as rs
import numpy as np
import cv2
import math
''' 
设置
'''
pipeline = rs.pipeline()  # 定义流程pipeline，创建一个管道
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)  # 配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # 配置color流

# config.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# config.enable_stream(rs.stream.depth,  1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipe_profile = pipeline.start(config)  # streaming流开始

# 创建对齐对象与color流对齐
align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)  # rs.align 执行深度帧与其他帧的对齐

''' 
获取对齐图像帧与相机参数
'''


point = (400, 300)
def show_distance(event, x, y, args, params):
    global point
    point = (x, y)
def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧，获取颜色和深度的框架集
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐

    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

    #### 获取相机参数 ####
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参

    #### 将images转为numpy arrays ####
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame


''' 
获取随机点三维坐标
'''


def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
    # print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return dis, camera_coordinate

cv2.namedWindow("RealSence")

cv2.setMouseCallback("RealSence", show_distance)

if __name__ == "__main__":
    while True:
        ''' 
        获取对齐图像帧与相机参数
        '''
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()  # 获取对齐图像与相机参数

        ''' 
        获取随机点三维坐标
        '''
        depth_pixel = [point[0], point[1]]  # 设置随机点，以相机中心点为例

        dis, camera_coordinate = get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)
        print('depth: ', dis)  # 深度单位是mm
        print('camera_coordinate: ', camera_coordinate)

        ''' 
        显示图像与标注
        '''
        #### 在图中标记随机点及其坐标 ####
        cv2.circle(img_color, (point[0],point[1]), 8, [255, 0, 255], thickness=-1)

        Z = dis
        dis = math.sqrt(((camera_coordinate[0]) ** 2) + ((camera_coordinate[1]) ** 2) + ((camera_coordinate[2]) ** 2))

        dis = round(dis*1000, 3)
        camera_coordinate[0]=round(camera_coordinate[0]*1000, 3)
        camera_coordinate[1]=round(camera_coordinate[1]*1000, 3)
        camera_coordinate[2]=round(camera_coordinate[2]*1000, 3)
        cv2.putText(img_color, "Dis:" + str(dis) + " mm", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.1, [0, 0, 255])
        cv2.putText(img_color, "X:" + str(camera_coordinate[0]) + " mm", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                    [255, 0, 0])
        cv2.putText(img_color, "Y:" + str(camera_coordinate[1]) + " mm", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                    [255, 0, 0])
        cv2.putText(img_color, "Z:" + str(camera_coordinate[2]) + " mm", (80, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                    [255, 0, 0])

        #### 显示画面 ####
        cv2.putText(img_color, "D:{}".format(dis), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 1,
                    (0, 0, 255), 1)
        cv2.putText(img_color, "X: {}, Y:{}, Z:{}".format(camera_coordinate[0], camera_coordinate[1],Z), (point[0], point[1] - 45),
                    cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)

        cv2.imshow('RealSence', img_color)
        # cv2.imshow("Color frame", img_color)
        key = cv2.waitKey(1)
