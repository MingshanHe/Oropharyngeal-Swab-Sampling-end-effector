#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from math import fabs
from operator import truediv
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
import tensorflow as tf 
import cv2
import numpy as np
import os

# Load Model
def NNload(path,GPU=True):
    if not GPU:
        os.environ["CUDA_VISIBLE_DEVICES"]="-1"
    else:
        physical_device = tf.config.experimental.list_physical_devices("GPU")
        tf.config.experimental.set_memory_growth(physical_device[0], True)
    return tf.keras.models.load_model(path)

# Model Predict
def NNpredict(model,img,threshold=0.5):
   # assert len(img.shape) == 2
    img = np.expand_dims(img,axis=-1)
    img = np.expand_dims(img,axis=0)
    pred = model.predict(img)[0]
    pred[pred <= threshold] = 0
    pred[pred > threshold] = 255
    return pred

def NNpostprocess(img,pred,visualize=True):
    x=y=10000
    radius=0
    pred = np.uint8(pred)
    center=[0,0]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    pred = cv2.morphologyEx(pred, cv2.MORPH_CLOSE, kernel,iterations=3)    #closed calculation
    contours,h= cv2.findContours(pred,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0  :
        return 10000,10000, img,pred,radius

    center=[400,270]
    area = cv2.contourArea(contours[0])
    if area < 60:
        return 10000,10000, img,pred,radius

    cnt_max=[]
    area_max=0
    for i in range (len(contours)):
        area = cv2.contourArea(contours[i])
        if  area_max < area:
            area_max = area
            cnt_max=contours[i]
    cnt_max=np.array(cnt_max)
    (x,y),radius = cv2.minEnclosingCircle(cnt_max)
    center = (int(x),int(y))
    radius = int(radius)
    # print(radius)
    cv2.circle(img,center,radius,255,1)
    cv2.circle(img,center,3,[0,0,255],3) 
    return x,y,img,pred,radius


def main():
    global center, radius
    Twist_Pub = rospy.Publisher('/twist', Wrench, queue_size=2)
    rospy.init_node('image_process', anonymous=True)
    rate = rospy.Rate(125) # 100hz

    flag=0
    model = NNload('/home/rl/catkin_ws/src/image_process/model/fcn.h5')
    from pathlib import Path
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(4,960) #setting pixels
    cap.set(3,1280)
    r=0

    falg = True
    while not rospy.is_shutdown() and falg:
        ret,img = cap.read()
        img = cv2.resize(img,(640,480))
        pred = NNpredict(model,img)
        img = cv2.flip(img,1,dst=None) #vertical mirror
        pred = cv2.flip(pred,1,dst=None) #vertical mirror
        x_,y_,img,pred,radius_ = NNpostprocess(img,pred)
        # center=[400,270]
        print('center is: (', x_,',',y_,') and radius is: (',radius_,')')
        if input('Input "Enter" to choose this center') == '':
            falg = False
            center=[x_,y_]
            radius = radius_
    fourcc1 = cv2.VideoWriter_fourcc(*'XVID')
    fourcc2 = cv2.VideoWriter_fourcc(*'XVID')
    out1 = cv2.VideoWriter('output1.avi',fourcc1, 30.0, (640,480))
    out2 = cv2.VideoWriter('output2.avi',fourcc2, 30.0, (640,480))

    while not rospy.is_shutdown():

        ret,img = cap.read()
        img = cv2.resize(img,(640,480))
        pred = NNpredict(model,img)
        img = cv2.flip(img,1,dst=None) #vertical mirror
        out1.write(img)
        pred = cv2.flip(pred,1,dst=None) #vertical mirror
        x,y,img,pred,radius_ = NNpostprocess(img,pred)
        out2.write(img)
        # center=[400,270]


        if radius>10  and y-center[1]>20:
            cv2.putText(img, "go", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        elif radius>10 and center[1]-y>20:
            cv2.putText(img, "down", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if radius>10 and x-center[0]>20:
            cv2.putText(img, "left", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        elif radius>10 and center[0]-x>20:
            cv2.putText(img, "right", (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if radius>10 and radius-r>10:
            cv2.putText(img, "bottom", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        elif radius>10 and r-radius>10:
            cv2.putText(img, "up", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if (fabs(center[0]-x)<40) and (fabs(center[1]-y)<40):

            wrench = Wrench()
            wrench.force.x = (center[0]-x)/120
            wrench.force.y = (center[1]-y)/120
            wrench.force.z = (radius-radius_)/10
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = 0
            Twist_Pub.publish(wrench)
            rate.sleep()
        else:
            wrench = Wrench()
            wrench.force.x = (center[0]-x)/120
            wrench.force.y = (y-center[1])/120
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = 0
            Twist_Pub.publish(wrench)
            rate.sleep()



        cv2.imshow('pre', pred)
        cv2.imshow('img', img)
        k = cv2.waitKey(1)
        # q key to quit()
        if k & 0xff == ord('q'):
            break
        elif  k & 0xff == ord('s'):
            center=[x,y]
            r=radius

        

        # twist.linear.x = x - center[0]




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass









# if __name__ == "__main__":

#     cap.release()
#     cv2.destroyAllWindows()