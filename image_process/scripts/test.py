import cv2
 
cap = cv2.VideoCapture(0) #视频进行读取操作以及调用摄像头
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
width = 640
ret = cap.set(3, width)
height = 480
ret = cap.set(4, height)
 
fourcc = cv2.VideoWriter_fourcc(*'XVID')
 
out = cv2.VideoWriter('out.avi', fourcc, 20.0, (width, height))


while cap.isOpened(): #判断视频读取或者摄像头调用是否成功，成功则返回true。
    ret, frame = cap.read()
    if ret is True:
        print('frame shape:',frame.shape)
        frame = cv2.resize(frame, (640, 480))
        out.write(frame)
 
        cv2.imshow('frame', frame)
 
    else:
        break
 
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
 
cap.release()
out.release()
cv2.destroyAllWindows()
