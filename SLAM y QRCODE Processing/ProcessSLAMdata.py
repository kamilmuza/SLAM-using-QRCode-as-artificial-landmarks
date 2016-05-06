__author__ = 'Kamil'
import cv2
import numpy as np
import zbar
import math
import time
from operator import mod, neg
import sys
import os
import tools
import drawing
import tools
#import and configure logging
import logging
logging.basicConfig(filename='odometry.log',level=logging.DEBUG,format='%(asctime)s - %(levelname)s - %(message)s')
#finish configure logging

x = None
class Stream:
    global data
    def __init__(self, k, d,qr, web):
        self.intrinsic = k
        self.dist_coeff = d
        self.qr_points = qr
        self.stream_web = web
        self.capture = cv2.VideoCapture()
        self.img = 0
        self.ids = []
        self.qrPL = 0
        self.max = 0
        self.symbol_flag = False       

    def open_stream(self):
        cv2.namedWindow("Streaming...")
        self.capture.open(self.stream_web)

    def close_stream(self):
        self.capture.release()
        cv2.destroyAllWindows()

    def process_frame(self):
        (grabbed, self.img) = self.capture.read()
        if not grabbed:
            return False
        return True

    def detect_symbols(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        (height, width) = gray.shape
        raw = str(gray.data)
        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')
        image1 = zbar.Image(width, height, 'Y800', raw)
        scanner.scan(image1)
        amount = 0
        amount = int(amount)
        data = []
        corners = []
        ids = []
        strip = []
        new = []
        self.symbol_flag = False
        for symbol in image1:
            self.symbol_flag = True
            #Amount of symbols in an image
            data = np.resize(data, amount)
            ids = np.resize(ids,amount)
            data = np.append(data, symbol.data)
            dec = data[amount]
            strip = dec.split(';')
            qrID = strip[0]
            qrPL = strip[1]
            qrID = int(qrID)
            qrPL = float(qrPL)
            self.qrPL = qrPL
            ids = np.append(ids, qrID)
            self.ids = ids
            TLC, BLC, BRC, TRC = [item for item in symbol.location]
            box = np.array([[TLC],[BLC],[BRC],[TRC]])
            box = np.reshape(box,(1,4,2))
            new = np.resize(new,(amount,4,2))
            new = np.insert(new,amount,box,axis = 0)
            amount = amount + 1
        self.max = amount
        return new

    def bounding_box(self,box,img):
        rows = box.shape[0]
        for a in range(rows):
            TLC = box[a][0]
            TRC = box[a][1]
            BRC = box[a][2]
            BLC = box[a][3]
            contour = np.int32([TLC, TRC, BRC, BLC])
            cv2.drawContours(img, [contour],-1, (0, 255, 0), 2)
        return img

    def distance_measure(self, box, img):
        rows = box.shape[0]
        focal_length = self.intrinsic[0, 0]
        dist_arr = np.zeros((rows,1))
        for a in range(rows):
            TLC = box[a][0]
            TRC = box[a][3]
            qrpixelwidth = math.sqrt((TLC[0]-TRC[0])**2+(TLC[1]-TRC[1])**2)
            distance = (self.qrPL * focal_length) / qrpixelwidth
            dist_arr = np.resize(dist_arr,(a,1))
            dist_arr = np.append(dist_arr,distance)
        return dist_arr

    def camera_poses(self,box):
        rows = box.shape[0]        
        poses = np.zeros((3,0))
        for a in range(rows):           
            rtVec, rVec, tVec = cv2.solvePnP(self.qr_points, box[a], self.intrinsic, self.dist_coeff)
            rot = cv2.Rodrigues(rVec)[0]
            rot_mat = np.transpose(rot)
            camera_pose = np.dot(-rot_mat, tVec)
            poses = np.append(poses,camera_pose, axis = 1)
        return poses

    def solve_pnp_distance(self, box,poses):
        rows = box.shape[0]
        distances = np.zeros((rows,1))
        for a in range(rows):
            pose = poses[:3,a]
            distance = pose[2]
            if distance < 0:
                distance = neg(distance)
            distances = np.resize(distances,(a,1))
            distances = np.append(distances,distance)

        return distances

    def object_angle(self,box):
        rows = box.shape[0]
        angles = np.zeros((rows,1))
        for a in range(rows):
            rtVal, rVec, tVec = cv2.solvePnP(self.qr_points, box[a], self.intrinsic, self.dist_coeff)
            rot = cv2.Rodrigues(rVec)[0]
            value = rot[2][1]**2+rot[2][2]**2
            rads = math.acos(math.sqrt(value))
            angle = (180 * rads)/math.pi
            angles = np.resize(angles,(a,1))
            angles = np.append(angles,angle)
        return angles

    def left_or_right(self, box, poses):
        for a in range(box.shape[0]):
            pose = poses[:3,a]
            if pose[0] < 0:
                return "Looking Qr Code from the Right"
            elif pose[0] > 0:
                return "Looking Qr Code from the Right"

    def bearing(self, box):
        Cx = self.intrinsic[0,2] # Center of projection
        rows = box.shape[0]
        focal = self.intrinsic[0,0]
        thetas = np.zeros((rows,1))
        float1 = float(54)
        float2 = float(640)
        coef = float1/float2
        for a in range(rows):
            TLC = box[a][0]
            TRC = box[a][3]
            TLCx = TLC[0]
            TRCx = TRC[0]
            dif = TRCx - TLCx
            qr_center = dif/2
            qr_center = TLCx + qr_center
            # length = Cx - qr_center
            length = 320 - qr_center
            theta = coef * length
            theta = np.deg2rad(theta)
            #rads = math.atan(length/focal)
            #theta = (180 * rads)/math.pi
            thetas = np.resize(thetas,(a,1))
            thetas = np.append(thetas,theta)

        return thetas

    def draw_screen(self,img, box,angles, distances,dist, thetas ):
        font = cv2.FONT_HERSHEY_SIMPLEX
        y = img.shape[0] - 20
        x = img.shape[1] - 8
        for a in range(box.shape[0]):            
            cv2.putText(img, "QR ID :%d" % self.ids[a], (self.img.shape[1]-x, self.img.shape[0]-y), font, 0.5, (0, 255, 0), 1)
            #cv2.putText(img, "Ang :%.3fd" % angles[a], (self.img.shape[1]-x, self.img.shape[0]-y+30), font, 0.5, (0, 255, 0), 1)
            cv2.putText(img, "SolvePnP:%.3fm" % distances[a], (self.img.shape[1]-x, self.img.shape[0]-y+60), font, 0.5, (0, 255, 0), 1)
            cv2.putText(img, "Dist:%.3fm" % dist[a], (self.img.shape[1]-x, self.img.shape[0]-y+90), font, 0.5, (0, 255, 0), 1)
            #cv2.putText(img, "Bear:%.3fr" % thetas[a], (self.img.shape[1]-x, self.img.shape[0]-y+120), font, 0.5, (0, 255, 0), 1)
            x = x - 110           
        return img

    def take_photo(self):
        cv2.imwrite("I:\pic.png",self.img)
        return True

    def use_photo(self):
        img = cv2.imread("I:\pic.png")
        return img

def main():   
    while proc_stream.process_frame():
        cv2.imshow("Streaming...", proc_stream.img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        tosend = []
        proc_stream.take_photo()
        img = proc_stream.use_photo()
        new = proc_stream.detect_symbols(img) #This box contains QR ID and Feature Max
        if proc_stream.symbol_flag is True:
            img = proc_stream.bounding_box(new,img)
            dist = proc_stream.distance_measure(new,img)
            poses = proc_stream.camera_poses(new)            
            angles = proc_stream.object_angle(new)
            d = proc_stream.distance_measure(new, img)
            distances = proc_stream.solve_pnp_distance(new,poses)
            L_o_R = proc_stream.left_or_right(new, poses)
            thetas = proc_stream.bearing(new)
            for a in range(proc_stream.max):
                qrid = proc_stream.ids[a]
                dist = distances[a]
                bear = thetas[a]
                send = np.array([[qrid],[dist],[bear]])             
                tosend = np.resize(tosend,(3,a))
                tosend = np.append(tosend,send, axis =1)
            print "Data:",tosend
            img = proc_stream.draw_screen(img, new, angles, distances,d, thetas)
            cv2.imshow("pic...", img)
            time.sleep(0.1)
    proc_stream.close_stream()    

def read_data(path):
    first = True
    data = []
    sensor = []
    i=0
    with open(path) as my_file:
        for line in my_file:
            tokens = line.split()
            i+=1
            #if 'rawODOMETRY' in tokens[0]:        #Alternate This!!!
            if 'ODOMETRY' in tokens[0]:          #Alternate this!!!
                if not first:
                    data.append({'sensor': sensor, 'odometry': odometry})
                    sensor = []
                if first:
                    first = False
                odometry = {'r1': float(tokens[1]), 't': float(tokens[2]), 'r2': float(tokens[3])}
    print "Lineas en new:%d"%i
    # return data[1:]
    return data

def read_raw(path):
    first = True
    data = []
    sensor = []
    with open(path) as my_file:
        for line in my_file:
            tokens = line.split()
            if 'rawODOMETRY' in tokens[0]:        #Alternate This!!!
            #if 'ODOMETRY' in tokens[0]:          #Alternate this!!!
                if not first:
                    data.append({'sensor': sensor, 'odometry': odometry})
                    sensor = []
                if first:
                    first = False
                odometry = {'x': float(tokens[1]), 'y': -1*float(tokens[2]), 'z': -1*float(tokens[3])}
                # odometry = {'x': float(tokens[1]), 'y': float(tokens[2]), 'z': float(0.785398163397 + float(tokens[3]))}
                # odometry = {'x': float(tokens[1]), 'y': float(tokens[2]), 'z': float(float(tokens[3])-0.785398163397)}
           
    # return data[1:]
    return data    

def process_images(path):
    tosend = []
    data = read_data("new.dat")
    global x
    output = open("data/sensor.dat",'w')
    #inputfile = open("odomotry.dat",'r')
    #newfile = open("slam",'w')   
    #file_list = os.listdir(path)
    #for line in file_list:
    #    print line
    for (step, reading) in enumerate(data):
        odometry = [reading['odometry']['r1'], reading['odometry']['t'], reading['odometry']['r2']]
        i = str('Pic_%03i.png' %step)  
        print "Processing %s" %i 
        #img = cv2.imread("I:/Raul/AI4R/testing/pics/%s"%line) 

        img = cv2.imread("I:/Raul/AI4R/testing/pics/%s"%i)
        new = x.detect_symbols(img) #This box contains QR ID and Feature Max       
        #time.sleep(0.5) 
        if x.symbol_flag is True:
            #x.symbol_flag = False
            img = x.bounding_box(new,img)
            #dist = x.distance_measure(new,img)
            poses = x.camera_poses(new)
            #degree = x.angle_measurement(new,poses)
            #angles = x.object_angle(new)
            distances = x.solve_pnp_distance(new,poses)
            #x.left_or_right(new, poses)
            output.write('%s %.7f %.7f %.7f\n' %('ODOMETRY',odometry[0], odometry[1] , odometry[2]))
            thetas = x.bearing(new)
            for a in range(x.max):             
                qrid = x.ids[a]
                dist = distances[a]
                bear = thetas[a]
                send = np.array([[qrid],[dist],[bear]])
                #tosend = np.resize(tosend,(3,a))
                #tosend = np.append(tosend,send, axis =1)
                output.write('%s %i %.7f %.7f\n' %('SENSOR',qrid, dist, bear))
                print send             
        elif x.symbol_flag is False:
            qrid = 0
            dist = 0
            bear = 0
            output.write('%s %.7f %.7f %.7f\n' %('ODOMETRY',odometry[0], odometry[1] , odometry[2]))
            output.write('%s %i %.7f %.7f\n' %('SENSOR',qrid, dist, bear))
            print "NO QR"
    output.close()

def process_raw_odometry():
    """
    Process raw odometry data from file to create rt1, tr, rt2 
    odometry model for SLAM
    This function generates a file containing odometry data structure
    for SLAM.
    """
    x_p = 0
    y_p = 0
    z_p = 0    
    out_file = open('new.dat','w')
    data = read_raw("raw_Odometry.dat") 
    for (step, reading) in enumerate(data):
        odometry = [reading['odometry']['x'], reading['odometry']['y'], reading['odometry']['z']]

        print odometry
        x = odometry[0]
        y = odometry[1]        
        z= odometry[2]
        #drawing.draw_state_for_me(step, x, y, z)
        z = tools.normalize_angle(z)
        rt_1 = math.atan2(y-y_p , x-x_p) - z_p
        t = math.sqrt((x - x_p)**2+(y - y_p)**2)    
        rt_2 = z - z_p - rt_1
        x_p = x
        y_p = y
        z_p = z
        out_file.write('%s %.7f %.7f %.7f\n' %('ODOMETRY', rt_1, t, rt_2))  
    out_file.close()    

def test_odometry():
    raw_file = "raw_Odometry.dat"
    process_file = 'new.dat'
    data = []
    rawdata = []
    x_p=0
    y_p=0
    z_p=0
    max_x=0
    max_y=0
    max_z=0
    min_x=0
    min_y=0
    min_z=0
    with open(process_file) as my_file:
        for line in my_file:
            tokens = line.split()
            if 'ODOMETRY' in tokens[0]:
                r1=float(tokens[1])
                t=float(tokens[2])
                r2=float(tokens[3])
                x=x_p + t*math.cos(z_p+r1)                            
                y=y_p + t*math.sin(z_p+r1)                            
                z=z_p+r1+r2
                x_p=x
                y_p=y
                z_p=z
                odometry = {'x': float(x), 'y': float(y), 'z': float(z)}
                data.append(odometry)
    with open(raw_file) as my_file:
        for line in my_file:
            tokens = line.split()
            if 'rawODOMETRY' in tokens[0]:                            
                odometry = {'x': float(tokens[1]), 'y': -1*float(tokens[2]), 'z': -1*float(tokens[3])}
                rawdata.append(odometry)                
    for (step,(d,rd)) in enumerate(zip(data,rawdata)):
        print step
        dx=d['x']-rd['x']
        dy=d['y']-rd['y']
        dz=d['z']-rd['z']
        drawing.draw_state_for_me(step, rd['x'], rd['y'], rd['z'],'/testo/')
        if dx > max_x:
            max_x=dx
        if dx < min_x:
            min_x=dx
        
        if dy > max_y:
            max_y=dy
        if dy < min_y:
            min_y=dy
        
        if dz > max_z:
            max_z=dz
        if dz < min_z:
            min_z=dz
        
        # logging.info("Step:%d\n\n"%step)
        # logging.info("dx:%0.5f  dy:%0.5f  dz:%0.5f"%(dx,dy,dz))
        # logging.info("x:%0.5f  y:%0.5f  z:%0.5f"%(d['x'],d['y'],d['z']))
        # logging.info("rx:%0.5f  ry:%0.5f  rz:%0.5f"%(rd['x'],rd['y'],rd['z']))
    print "max"
    print max_x,max_y,max_z
    print "min"
    print min_x,min_y,min_z

def test_movement():
    data = tools.read_data('data/sensor.dat')
    x_p=0
    y_p=0
    z_p=0
    for (step, reading) in enumerate(data):
        print step
        # if step == 10:
        # break
        r1 = reading['odometry']['r1']
        r2 = reading['odometry']['r2']
        t = reading['odometry']['t']
        x=x_p + t*math.cos(z_p+r1)                            
        y=y_p + t*math.sin(z_p+r1)                            
        z=z_p+r1+r2
        x_p=x
        y_p=y
        z_p=z
        drawing.draw_state_for_me(step, x, y, z,'/testm/')
    
if __name__ == "__main__":
    K = np.array([[608.14578, 0, 292.31368],
              [0, 608.82584, 261.47420],
              [0, 0, 1]],dtype = np.float64)  # 3x3 Calibration intrinsic matrix
    dist_coeff = np.array([ [0.02232355], [-0.11265588], [0.00247792], [-0.00201052], [0.22786218]],dtype=np.float64 )  # 5x1 Distortion vector
    qr_points = np.float32([[-0.0975, -0.0975, 0],[-0.0975, 0.0975, 0],[0.0975, 0.0975, 0],[0.0975, -0.0975, 0]])
    stream_web = "http://192.168.112.100:8080/stream/video.mjpeg"
    sample_time = 0.5
    i = 0
    qrid = 0
    dist = 0
    bear = 0
    path = "I:/Raul/AI4R/testing/pics"
    print "Beginning"
    proc_stream = Stream(K, dist_coeff, qr_points, stream_web)    
    proc_stream.open_stream()
    main()
    #process_raw_odometry()
    #process_images(path)
    #test_odometry()
    #test_movement()
