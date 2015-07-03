import numpy as np
import cv2
import sys
import math
import time
from threading import Thread
ix,iy = -1, -1
def get_mouse_value(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        ix, iy = x,y

class Traceball(Thread):
    def __init__(self,videoNum):
        super(Traceball,self).__init__()
        self.videoNum=videoNum
        self.xyr=(-1,-1,-1)
        self.tracking=True
    def get(self):
         return self.xyr
    def turnoff(self):
         self.tracking=False
    def run(self):
        cam = cv2.VideoCapture(self.videoNum)
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
        cv2.namedWindow('img')
        cv2.setMouseCallback('img',get_mouse_value)

        while self.tracking:
            ret, img = cam.read()
            if ret == True:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
                hsv = cv2.medianBlur(hsv,51)
                
                threshold = cv2.inRange(hsv, ORANGE_MIN, ORANGE_MAX)
                kernel = np.ones((2,2),np.uint8)
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
                            #print cx,cy,epsilon,max_area,'\n'
                            self.xyr= (cx,cy,epsilon)
                            print self.xyr
    
                else:
                    pre_cx, pre_cy = -1, -1
                cv2.imshow('threshold', threshold)    
                cv2.imshow('img', hsv)

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
                       ORANGE_MIN=hsv[iy][ix]-np.array([10,50,50])
                       ORANGE_MAX=hsv[iy][ix]+np.array([10,50,50])
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
                if ch == ord('p'):
                    print "-----write range to file-----"
                    text_file = open("range.txt", "w")
                    text_file.write("Range_MIN: {0}".format(ORANGE_MIN))
                    text_file.write("Range_MAX: {0}".format(ORANGE_MAX))
                    text_file.close()
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

        print "exit"
                 

def get_trace(videoNum):
    th = Traceball(videoNum)
    th.start()
    count=0
    while True:
        count+=1
        (x, y, r) = th.get()
        time.sleep(0.1)
        print "get ",x,y,r
        if count>10:
            th.turnoff()
            break
        

    

if __name__ == '__main__':
    videoNum = int(sys.argv[1])
    
    get_trace(videoNum)
    
 
