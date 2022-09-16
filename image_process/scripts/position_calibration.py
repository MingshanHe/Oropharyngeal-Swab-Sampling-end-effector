
import cv2
import urx
import logging
import math
import numpy as np
import time
import os
import copy
import numpy as np
import os
import readchar
import rospy
import sys
import yaml
from geometry_msgs.msg import TransformStamped
from transforms3d.quaternions import *
#############################################
logging.basicConfig(level=logging.WARN)
rob = urx.Robot("192.168.0.103")
#rob = urx.Robot("localhost")
rob.set_tcp((0,0,0,0,0,0))
rob.set_payload(0.5, (0,0,0))
l = 0.05
v = 0.1
a = 0.03

# -0.009
# -0.0029
# 0.587
points = np.array([-0.009,-0.0029,0.587])

cd = os.path.dirname(os.path.realpath(sys.argv[0]))
data_dir = os.path.join(cd, '..', 'env')
with open(os.path.join(data_dir, '/home/rl/catkin_ws/vscode/camera_pose_aligned_plane.yml'), 'r') as f:
    camera_pose_dict = yaml.load(f, Loader=yaml.FullLoader)



# -0.0111
# -0.0036
# 0.724

points = np.array([-0.009,-0.0029,0.587])
depth_frame_pose = [0,0,0,0.5,-0.5,0.5,-0.5]
depth_optical_frame_pose = [0,0,0,1,0,0,0]
pose = rob.getl()
i=0
points = (np.matmul(quat2mat(depth_frame_pose[3:]), points.T)).T + depth_frame_pose[:3]    
points = (np.matmul(quat2mat(depth_optical_frame_pose[3:]), points.T)).T + depth_optical_frame_pose[:3]
pose = rob.getl()
cam_ori = camera_pose_dict["cam_{}".format(i+1)]["orientation"]
points = (np.matmul(quat2mat(cam_ori), points.T)).T + camera_pose_dict["cam_{}".format(i+1)]["position"]
print(points)
pose = [-points[0],-points[1],points[2],pose[3],pose[4],pose[5]]
print(pose)
rob.movel(pose, acc=a, vel=v)

