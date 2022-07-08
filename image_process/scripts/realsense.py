#!/usr/bin/env python
# -*- coding: utf-8 -*-


# Import required Python code.

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import dlib
import numpy as np
# import pyrealsense2 as rs

LIP_MARGIN = 0.3                # Marginal rate for lip-only image.
RESIZE = (64,64)                # Final image size
CROPPED_RADIUS = 30             # Cropped Radius

# Face detector and landmark detector
face_detector = dlib.get_frontal_face_detector()   
landmark_detector = dlib.shape_predictor("/home/rl/catkin_ws/src/image_process/model/shape_predictor_68_face_landmarks.dat")

def shape_to_list(shape):
	coords = []
	for i in range(0, 68):
		coords.append((shape.part(i).x, shape.part(i).y))
	return coords

def callback(data1,data2):

    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(data1, 'bgr8')
    depth_image = bridge.imgmsg_to_cv2(data2, '16UC1')

    color_image_size = color_image.shape
    # print(sp) : (480, 640, 3)

    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)   # Convert image into grayscale
    frame_buffer=gray                 # Add image to the frame buffer
    frame_buffer_color=color_image
    landmark_buffer = []        
    image=frame_buffer      
    face_rects = face_detector(image,1)

    if len(face_rects) < 1:                
        print("No face detected: ")
        return
    if len(face_rects) > 1:                  # Too many face detected
        print("Too many face: ")
        return

    rect = face_rects[0]                    # Proper number of face
    landmark = landmark_detector(image, rect)   # Detect face landmarks
    landmark = shape_to_list(landmark)
    cropped_buffer = []

    lip_landmark = landmark[48:68]                                          # Landmark corresponding to lip
    lip_x = sorted(lip_landmark,key = lambda pointx: pointx[0])             # Lip landmark sorted for determining lip region
    lip_y = sorted(lip_landmark, key = lambda pointy: pointy[1])
    
    x_add = int((-lip_x[0][0]+lip_x[-1][0])*LIP_MARGIN)                     # Determine Margins for lip-only image
    y_add = int((-lip_y[0][1]+lip_y[-1][1])*LIP_MARGIN)
    
    crop_pos = (lip_x[0][0]-x_add, lip_x[-1][0]+x_add, lip_y[0][1]-y_add, lip_y[-1][1]+y_add)   # Crop image
    cropped = frame_buffer_color[crop_pos[2]:crop_pos[3],crop_pos[0]:crop_pos[1]]
    cropped_depth = depth_image [crop_pos[2]:crop_pos[3],crop_pos[0]:crop_pos[1]]
    cropped_depth = cv2.applyColorMap(cv2.convertScaleAbs(cropped_depth, alpha=0.03), cv2.COLORMAP_JET)
    cropped = cv2.resize(cropped,(RESIZE[0],RESIZE[1]),interpolation=cv2.INTER_CUBIC)        # Resize
    cropped_buffer.append(cropped)

    cropped_center = (int((crop_pos[0]+crop_pos[1])/2),int((crop_pos[2]+crop_pos[3])/2))
    cv2.circle(color_image, cropped_center, CROPPED_RADIUS, 255, 1)

    cropped_distance = depth_image[int((crop_pos[0]+crop_pos[1])/2),int((crop_pos[2]+crop_pos[3])/2)]
    print(cropped_distance)



    cv2.imshow("cropped", cropped)
    cv2.imshow("cropped_depth", cropped_depth)

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    # Stack both images horizontally
    images = np.hstack((color_image, depth_colormap))
    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    key = cv2.waitKey(1)





    # cv2.imshow('color_image',color_image)
    # cv2.waitKey(1000)
    # c_x = 320
    # c_y = 240
    # real_z = depth_image[c_y, c_x] * 0.001  
    # real_x = (c_x- ppx) / fx * real_z
    # real_y = (c_y - ppy) / fy * real_z
    # rospy.loginfo("potion:x=%f,y=%f,z=%f",real_x,real_y,real_z) #输出图像中心点在相机坐标系下的x,y,z
 
if __name__ == '__main__':
    global fx, fy, ppx, ppy #相机内参
    fx = 609.134765 
    fy = 608.647949
    ppx = 312.763214
    ppy = 240.882049
 
    rospy.init_node('get_image', anonymous=True)
 
    color = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 1, allow_headerless=True)
    color_depth.registerCallback(callback)  
    #同时订阅/camera/color/image_raw和/camera/aligned_depth_to_color/image_raw话题，并利用message_filters实现话题同步，共同调用callback
    rospy.spin()
