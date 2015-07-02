#encoding:utf-8
import numpy as np
import cv2
import sys
import math
from matplotlib import pyplot as plt
ix,iy = -1, -1

def get_mouse_value(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        ix, iy = x,y


if __name__ == '__main__':

    cam = cv2.VideoCapture(0)#读取视频
    ret, prev = cam.read()
    prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    show_img = False
    hsv_set = False
    max_area=-1
    ORANGE_MIN = np.array([0, 0, 0],np.uint8)
    ORANGE_MAX = np.array([255, 255, 255],np.uint8)
    cx=-1
    cy=-1
    px,py=-1,-1
    pre_cx, pre_cy=-1,-1
    pre_area=-1
    timg = np.zeros((480,640,3), np.uint8)
    cv2.namedWindow('img')
    cv2.setMouseCallback('img',get_mouse_value)

    while True:
        ret, img = cam.read()#读取视频的下一帧作为光流输入的当前帧
        if ret == True:#判断视频是否结束
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            hsv = cv2.medianBlur(hsv,51)
            if cv2.cv.WaitKey(10)==27:
                break
            
            threshold = cv2.inRange(hsv, ORANGE_MIN, ORANGE_MAX)
            cimg = cv2.cvtColor(threshold,cv2.COLOR_GRAY2BGR)
            kernel = np.ones((2,2),np.uint8)
            '''
            edges = cv2.Canny(hsv,100,50)
            cv2.imshow("edges", edges)
            contours,hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours)>0:
                cv2.drawContours(hsv, contours,- 1 ,( 0 , 255 , 0 ), 3)
            for h,cnt in enumerate(contours):
                mask = np.zeros(gray.shape,np.uint8)
                cv2.drawContours(hsv,[cnt],0,(255,0,0),-1)
                mean = cv2.mean(hsv,mask = mask)
            '''
            #gradient = cv2.inRange(gradient, np.array([2,2,2]), np.array([255,255,255]))
            erosion = cv2.dilate(threshold,kernel,iterations = 1)
            contours,hierarchy = cv2.findContours(erosion, 1, 2)
            threshold = cv2.cvtColor(threshold,cv2.COLOR_GRAY2BGR)
            max_area=0
            max_cnt=0
            if hsv_set is False and len(contours) >0 :
                for i in contours:
                    cnt = i-1
                    M = cv2.moments(cnt)
                    if cv2.contourArea(cnt)>max_area:
                        max_area=cv2.contourArea(cnt)
                        max_cnt=cnt
                M = cv2.moments(max_cnt)
                if M['m00']>0 :
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    if (pre_cx<0 and pre_cy<0)or(math.fabs(cx-pre_cx)<20 and 
                        math.fabs(cx-pre_cy),20 and 
                        max_area>1000 or 
                        math.fabs(max_area-pre_area)<10000):
                        pre_cx=cx
                        pre_cy=cy
                        pre_area=max_area
                        cv2.circle(threshold,(cx,cy),2,(0,0,255),3)
                        epsilon = 0.1*cv2.arcLength(max_cnt,True)
                        #TODO!
                        print cx,cy,epsilon,max_area,'\n'                        

            else:
                pre_cx, pre_cy = -1, -1
            cv2.imshow('threshold', threshold)    
            cv2.imshow('img', hsv)

            prevgray = gray
            

            ch = 0xFF & cv2.waitKey(5)
            if ch == ord('z'):
                print "set"
                hsv_set=True
            if ch == ord('x'):
                print "set off"
                hsv_set=False
            if ch == ord('c'):
                print "reset"
                pre_x,pre_y=-1,-1
                if(ix>0 and iy>0):
                   ORANGE_MIN=hsv[iy][ix]-np.array([2,2,2])
                   ORANGE_MAX=hsv[iy][ix]+np.array([2,2,2])
            if ch == ord('q'):
                break
            if ch == ord('1'):
                show_img = not show_img
                print 'show colored on/offq'
            if ch == ord('s'):
                cv2.imwrite('threshold.jpg', threshold)
                cv2.imwrite('hsv.jpg', hsv)
                cv2.imwrite('img.jpg', img)
                print '--save--'
            if hsv_set:
                if ix <0 or iy<0 or (ix==px and iy==py):
                    print "click"
                    continue
                print ix,iy,hsv[iy][ix],ORANGE_MIN,ORANGE_MAX
                if (ORANGE_MIN[0]>hsv[iy][ix][0]):
                    ORANGE_MIN[0]=hsv[iy][ix][0]
                elif(ORANGE_MAX[0]<hsv[iy][ix][0]):
                    ORANGE_MAX[0]=hsv[iy][ix][0]
                if (ORANGE_MIN[1]>hsv[iy][ix][1]):
                    ORANGE_MIN[1]=hsv[iy][ix][1]
                elif(ORANGE_MAX[1]<hsv[iy][ix][1]):
                    ORANGE_MAX[1]=hsv[iy][ix][1]
                if (ORANGE_MIN[2]>hsv[iy][ix][2]):
                    ORANGE_MIN[2]=hsv[iy][ix][2]
                elif(ORANGE_MAX[2]<hsv[iy][ix][2]):
                    ORANGE_MAX[2]=hsv[iy][ix][2]
                px=ix
                py=iy
                print px,py,hsv[iy][ix],ORANGE_MIN,ORANGE_MAX,'\n'
                
            if show_img:
                cv2.imshow('img', img)


    cv2.destroyAllWindows()             

