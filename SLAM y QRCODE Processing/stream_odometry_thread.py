__author__ = 'admin'
import threading
import time

import cv2
import math
flagO = False
stream_web = "http://192.168.112.100:8080/stream/video.mjpeg"
output = open("odomotry.dat",'w')
x = 5
y = 4
theta = 6
samples = 0
lap = 0
lap1 = 0

class Stream(threading.Thread):
    def __init__(self,web):
        threading.Thread.__init__(self)
    def run(self):
        global lap
        global lap1

        a = 0
        time1 = 0
        time2 = 0
        capture = cv2.VideoCapture()
        capture.open(stream_web)
        while a < 100:
            time1 = time.time()
            grabbed ,img = capture.read()
            timed1 = time.clock()
            path = 'I:/Raul/AI4R/testing/pics/Pic%.5f.png' %timed1
            time.sleep(0.3)
            cv2.imwrite(path,img)
            print "pic take it"
            time2 = time.time()
            a = a + 1
        return (time2 - time1)





class Odometry(threading.Thread):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        threading.Thread.__init__(self)
    def run(self):
        global lap
        global lap1
        time1 = time.time()
        x_p = 0
        y_p = 0
        theta_p = 0
        global samples
        while True:
            if samples is 100:
                break
            trans = math.sqrt((self.x-x_p)**2+(self.y-y_p)**2)
            rot_1 = math.atan2((self.y-y_p),(self.x-x_p)) - theta_p
            rot_2 = self.theta - theta_p - rot_1
            x_p = self.x
            y_p = self.y
            theta_p = self.theta
            timed2 = time.clock()
            time.sleep(0.309)
            output.write('%.5f %.5f %.5f %.5f\n' %(timed2, rot_1, trans, rot_2))
            samples = samples + 1
            print samples
        time2 = time.time()
        print "Odometry",(time2 - time1)
        print "Done"
        output.close()

thr1 = Stream(stream_web)
#thr2 = Odometry(x,y,theta)
thr1.start()
#thr2.start()