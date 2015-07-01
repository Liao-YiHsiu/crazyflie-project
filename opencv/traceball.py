#encoding:utf-8
import numpy as np
import cv2
import sys
import math


if __name__ == '__main__':

    cam = cv2.VideoCapture(0)#读取视频
    ret, prev = cam.read()
    prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    show_img = False
    ORANGE_MIN = np.array([100, 140, 100],np.uint8)
    ORANGE_MAX = np.array([120, 230, 255],np.uint8)
    cx=0
    cy=0
    while True:
        ret, img = cam.read()#读取视频的下一帧作为光流输入的当前帧
        if ret == True:#判断视频是否结束
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            hsv = cv2.medianBlur(hsv,81)
            if cv2.cv.WaitKey(10)==27:
                break
            thresheld = cv2.inRange(hsv, ORANGE_MIN, ORANGE_MAX)
            #thresheld = cv2.medianBlur(thresheld,5)
            cimg = cv2.cvtColor(thresheld,cv2.COLOR_GRAY2BGR)
            kernel = np.ones((2,2),np.uint8)
            erosion = cv2.erode(thresheld,kernel,iterations = 5)
            contours,hierarchy = cv2.findContours(erosion, 1, 2)
            max_area=0
            if len(contours) >0 :
                for i in contours:
                    cnt = i-1
                    M = cv2.moments(cnt)
                    if M['m00']>0 and cv2.contourArea(cnt)>max_area:
                        max_area = cv2.contourArea(cnt)
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        erosion = cv2.cvtColor(thresheld,cv2.COLOR_GRAY2BGR)
                        cv2.circle(erosion,(cx,cy),2,(0,0,255),3)
                        epsilon = 0.1*cv2.arcLength(cnt,True)
                        print cx,cy,epsilon,cv2.contourArea(cnt),'\n'
            cv2.imshow('flow', erosion)
            if show_img:
                cv2.circle(img,(cx,cy),2,(0,0,255),3)
                cv2.imshow('img', img)
            
            ch = 0xFF & cv2.waitKey(5)
            if ch == ord('q'):
                break
            if ch == ord('1'):
                show_img = not show_img
                print 'show colored on/offq'
            if ch == ord('2'):
                cv2.imwrite('erosion.jpg', erosion)
                cv2.imwrite('hsv.jpg', hsv)
                cv2.imwrite('img.jpg', img)
                print '--save--'
            


    cv2.destroyAllWindows()             

