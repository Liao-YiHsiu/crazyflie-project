#encoding:utf-8
import numpy as np
import cv2
import sys
import math
#import video #Opencv Python自带的读取

help_message = '''
USAGE: opt_flow.py [<video_source>]

Keys:
 1 - toggle HSV flow visualization
 2 - toggle glitch

'''

def draw_flow(img, flow, face_x,face_y,face_h,face_w,step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[face_y++face_h/4:face_y+3*face_h/4:step, face_x+face_w/4:face_x+3*face_w/4:step].reshape(2,-1)#以网格的形式选取二维图像上等间隔的点，这里间隔为16，reshape成2行的array
    fx, fy = flow[y,x].T#取选定网格点坐标对应的光流位移
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)#将初始点和变化的点堆叠成2*2的数组
    lines = np.int32(lines + 0.5)#忽略微笑的假偏移，整数化
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))#以初始点和终点划线表示光流运动
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)#在初始点（网格点处画圆点来表示初始点）
    return vis

def draw_hsv(flow):
    h, w = flow.shape[:2]
    fx, fy = flow[:,:,0], flow[:,:,1]
    ang = np.arctan2(fy, fx) + np.pi#得到运动的角度
    v = np.sqrt(fx*fx+fy*fy)#得到运动的位移长度
    hsv = np.zeros((h, w, 3), np.uint8)#初始化一个0值空3通道图像
    hsv[...,0] = ang*(180/np.pi/2)#B通道为角度信息表示色调
    hsv[...,1] = 255#G通道为255饱和度
    hsv[...,2] = np.minimum(v*4, 255)#R通道为位移与255中较小值来表示亮度
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)#将得到的HSV模型转换为BGR显示
    return bgr

def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = -flow
    flow[:,:,0] += np.arange(w)
    flow[:,:,1] += np.arange(h)[:,np.newaxis]
    res = cv2.remap(img, flow, None, cv2.INTER_LINEAR)#图像几何变换（线性插值），将原图像的像素映射到新的坐标上去
    return res

if __name__ == '__main__':
    import sys
    print help_message
    cascPath = sys.argv[1]
    faceCascade = cv2.CascadeClassifier(cascPath)
    videoNum = int(sys.argv[2])
    #video_capture = cv2.VideoCapture(videoNum)

    cam = cv2.VideoCapture(videoNum)#读取视频
    ret, prev = cam.read()#读取视频第一帧作为光流输入的当前帧֡
    #prev = cv2.imread('E:\lena.jpg')
    prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    reg = False
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()
    face_x=-1
    face_y=-1
    pre_x, pre_y, pre_h, pre_w=-1, -1,-1,-1
    reject_Levels=[1,2,3]
    level_Weights=[1,2,3]
    
	
    while True:
        ret, img = cam.read()#读取视频的下一帧作为光流输入的当前帧
        if ret == True:#判断视频是否结束
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if cv2.cv.WaitKey(10)==27:
                break
            #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            reg = True
            if reg == True:
                #cv2.CascadeClassifier.detectMultiScale(image, rejectLevels, levelWeights[, scaleFactor[, minNeighbors[, flags[, minSize[, maxSize[, outputRejectLevels]]]]]]) → objects
                faces = faceCascade.detectMultiScale(
                       gray,
                       reject_Levels,
                       level_Weights,
                       scaleFactor=1.1,
                       minNeighbors=5,
                       flags=cv2.cv.CV_HAAR_SCALE_IMAGE,
                       minSize=(30, 30),
                       maxSize=(500, 500),
                       outputRejectLevels=0
                       )

               
                flow = cv2.calcOpticalFlowFarneback(prevgray, gray, 0.5, 3, 15, 1, 5, 1.2, 0)#Farnback光流法
                #cv2.calcOpticalFlowFarneback(prev, next, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags[, flow]) 
                prevgray = gray#计算完光流后，将当前帧存储为下一次计算的前一帧
                # Draw a rectangle around the faces

                count=0
                
                now=gray
                for (x, y, w, h) in faces:                   
                    print "x=",x," y= ",y, "h=",h,"pre_x=",pre_x,"pre_y=",pre_y
                    if (pre_x<0 and pre_y<0)or(math.fabs(x-pre_x)<20 and math.fabs(y-pre_y)<20):
                        pre_x=x
                        pre_y=y
                        pre_w=w
                        pre_h=h

                if (pre_x>0 and pre_y>0):
                    now=draw_flow(gray, flow,pre_x,pre_y,pre_h,pre_w)
                    cv2.rectangle(now, (pre_x, pre_y), (pre_x+pre_w, pre_y+pre_h), (0, 255, 0), 2)
                cv2.imshow('flow', now)
                if show_hsv:
                    cv2.imshow('flow HSV', draw_hsv(flow))
    
                ch = 0xFF & cv2.waitKey(5)
                if ch == ord('q'):
                    break
                if ch == ord('1'):
                    show_hsv = not show_hsv
                    print 'HSV flow visualization is', ['off', 'on'][show_hsv]
            else:
               #print "find face"
               faces = faceCascade.detectMultiScale(
                       gray,
                       rejectLevels,
                       levelWeights,
                       scaleFactor=1.1,
                       minNeighbors=5,
                       minSize=(30, 30),
                       flags=cv2.cv.CV_HAAR_SCALE_IMAGE
                       )

               # Draw a rectangle around the faces
               for (x, y, w, h) in faces:
                   print len(faces)
                   cv2.rectangle(gray, (x, y), (x+w, y+h), (0, 255, 0), 2)
                   print "x=",x," y=",y
                   print "face_x=",face_x," face_y=",face_y
                   print math.fabs(face_x-x)," ",math.fabs(face_y-y)

               cv2.imshow('Video', gray)
               if (x>0 and y>0 and face_x<0 and face_y<0):
                  face_x = x
                  face_y = y
               #if (math.fabs(face_x-x)<30 and math.fabs(face_y-y)<30):
                #  reg=True


    cv2.destroyAllWindows()             

