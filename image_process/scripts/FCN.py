import tensorflow as tf 
import cv2
import numpy as np
import os

def NNload(path,GPU=False):
    if not GPU:
        os.environ["CUDA_VISIBLE_DEVICES"]="-1"
    else:
        physical_device = tf.config.experimental.list_physical_devices("GPU")
        tf.config.experimental.set_memory_growth(physical_device[0], True)
    return tf.keras.models.load_model(path)
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
    pred = cv2.morphologyEx(pred, cv2.MORPH_CLOSE, kernel,iterations=3)    #闭运算
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
center=[400,270]
if __name__ == "__main__":
    flag=0
    model = NNload('model/fcn.h5')
    from pathlib import Path
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(4,960) #设置分辨率
    cap.set(3,1280)
    r=0
    while 1:
        ret,img = cap.read()
#         img = img[30: 930, 80:1200]
        img = cv2.resize(img,(640,480))
        pred = NNpredict(model,img)
        img = cv2.flip(img,1,dst=None) #水平镜像
        pred = cv2.flip(pred,1,dst=None) #水平镜像
        x,y,img,pred,radius = NNpostprocess(img,pred)
        

#         if radius>10 and flag==0 and 280<x<340 and 180<y<280:
#             center=[x,y]
#             flag=1

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

            
            
            
        
        print(center,x,y)
#         print(x,y)
        #计算识别到的物体的中心距离触觉机械爪中心的距离，并进行移动
        cv2.imshow('pre', pred)
        cv2.imshow('img', img)
        k = cv2.waitKey(1)
        # q键退出
        if k & 0xff == ord('q'):
            break
        elif  k & 0xff == ord('s'):
            center=[x,y]
            r=radius
    cap.release()
    # 关闭窗口
    cv2.destroyAllWindows()