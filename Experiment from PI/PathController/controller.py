import math
import signal
import board
import time
import track
from lib_hmc6352 import HMC6352
import cv2
import thread

ENCODER_STEPS = 270.9

class Controller:
    def __init__(self):
        self.output = open('Odometry.dat', 'w')
        self.output2 = open('raw_Odometry.dat','w')
        self.arduino = board.Arduino()
        self.reference = track.Track()
        self.time_vector = []
        self.sample_time_vector = []
        self.x_position_vector = []
        self.x_position_dot_vector = []
        self.y_position_vector = []
        self.y_position_dot_vector = []
        self.z_position_vector = []
        self.z_position_dot_vector = []
        self.smooth_flag = True
        self.x_ref_vector = []
        self.y_ref_vector = []
        self.value1 = []
        #self.current1_vector = []
        self.reference1 = []
        self.value2 = []
        #self.current2_vector = []
        self.reference2 = []
        #self.file_name = '/home/pi/file.m'

        self.radius = 0.04911  # in meters
        self.distance = 0.38  # in meters
        self.constant_b = 0.05
        self.constant_k1 = 3.0
        self.constant_k2 = 3.0
        #self.constant_ki = 1.0
        #self.constant_kd = 1.0
        #self.constant_kc = 2.0
        self.sample_time = 0.05

        self.slamflag = False



        self.missed = 0
        self.accurate = 0
        self.max_elapsed = 0
        self.prev = 0
        self.finished = True
        self.count = 0
        #self.u1k1 = 0
        #self.u2k1 = 0
        #self.e1k1 = 0
        #self.e1k2 = 0
        #self.e2k1 = 0
        #self.e2k2 = 0
        self.x_position = 0
        self.y_position = 0
        self.z_position = 0
        self.x_p = 0
        self.y_p = 0
        self.theta_p = 0

        #self.compass = HMC6352()
        #self.compass_angle = 0

        self.list = range(2000)
        self.index = 0

        self.encoder1 = 0
        self.encoder2 = 0
        #self.current1 = 0
        #self.current2 = 0
        self.battery = 0

        # self.ccc = 0

        self.keys = [] # pressed key -> WASD movement
        thread.start_new_thread(self.move, ())

        #TODO: Revisar si hace falta
        self.prev_encoder1 = 0
        self.prev_encoder2 = 0
        self.prev_delta_encoder_1 = 0
        self.prev_delta_encoder_2 = 0

        self.globalPositionX = 0
        self.globalPositionY = 0
        self.globalPositionZ = 0
        self.samples = 0
        self.pic_stack = []

        self.safe_count = False
        self.safe_counter = 0
        self.timer_init()
        self.capture = cv2.VideoCapture(0)
        #self.capture.open("http://10.0.0.2:8080/stream/video.mjpeg")
        print "Camera Ready"

    def start_thread_slam(self):
        thread.start_new_thread(self.startSlam, ())

    def startSlam(self):
        while self.slamflag and self.samples < 300 :
            o = float(self.globalPositionZ)
            pi_2 = float(math.pi*2)
            z = o % pi_2 
            (grabbed ,img) = self.capture.read()
            time.sleep(0.2)    
            self.output2.write('%s %.7f %.7f %.7f\n' %('rawODOMETRY', self.globalPositionX, self.globalPositionY, self.globalPositionZ))          
            time1 =time.clock()                         
            self.pic_stack.append(img)       
            #cv2.imwrite(path,img)
            rot_1 = math.atan2((self.globalPositionY-self.y_p),(self.globalPositionX-self.x_p)) - self.theta_p
            trans = math.sqrt((self.globalPositionX-self.x_p)**2+(self.globalPositionY-self.y_p)**2)    
            rot_2 = z - self.theta_p - rot_1
            self.x_p = self.globalPositionX
            self.y_p = self.globalPositionY
            self.theta_p = z              
            self.output.write('%s %.7f %.7f %.7f\n' %('ODOMETRY', rot_1, trans, rot_2))  
            timed2 = time.clock()               
            self.samples = self.samples + 1
            char = "Sample number %d"%self.samples
            #time_sample = char+' '+str(self.samples)+"Take it at"+' '+str(timed2)  
            self.to_list(char)
                           
        print "Done"
        self.output2.close()
        self.output.close()
        self.capture.release()
        for i in range(self.samples):
            pic = self.pic_stack[i]
            cv2.imwrite("Pic_%03d.png" %i, pic)
            char2 = 'Copying sample number: %i to rpi SD CARD'%i  
            self.to_list(char2)
        print "All samples copied, experiment finished"   

        

    def set_parameter(self, parameter, value):
        self.__dict__[parameter] = value

    def to_list(self, m):
        self.list[self.index] = m
        self.index += 1
        if self.index >= 2000:
            self.index = 0

    def ask_status(self):
        encoder1, encoder2, battery = self.arduino.read_state()
        # self.arduino.reset_encoders()

        self.encoder1 = encoder1
        self.encoder2 = encoder2
        self.battery = battery

        self.to_list('status %d,%d,%c' % (self.encoder1, self.encoder2, self.battery))

    def update_compass_angle(self):
        self.compass_angle = self.compass.readData()

    def calibrate_compass(self):
        self.compass.userCalibration()

    def odometryTuner(self, relativeTimeInSecounds):
        a = []
        b = []
        CurrentTime = time.time()
        EndTime = CurrentTime + relativeTimeInSecounds
        self.rotate_inplace(4)
        # Cada 100ms
        while(CurrentTime < EndTime):
            time.sleep(.1)
            self.update_compass_angle()
            CurrentTime = time.time()
            a.append(self.compass_angle)
            b.append(self.z_position)

        # Cuando se acabe el tiempo
        self.stop()
        for i in xrange(len(b)):
            b[i] *= 57.2957795
            while abs(b[i]) > 360:
                b[i] += 360
            b[i] += 360
            b[i] += a[0]

            while b[i] > 360:
                b[i] -= 360
            print a[i], b[i]

    def movement_init(self, x, y, t):
        self.smooth_flag = True
        self.experiment_reset()

        delta_x = x - self.globalPositionX
        delta_y = y - self.globalPositionY

        beta = math.atan2(delta_y, delta_x)
        #theta_n = math.atan2(math.sin(self.globalPositionZ), math.cos(self.globalPositionZ))
        theta_n = self.globalPositionZ
        alpha = beta - theta_n
        l = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        xf_p = l * math.cos(alpha)
        yf_p = l * math.sin(alpha)

        track_parameters = {'x_planning': [0, xf_p],
                            'y_planning': [0, yf_p],
                            't_planning': [0, t],
                            'sample_time': self.sample_time}

        self.reference.generate(**track_parameters)

        self.time_vector = range(self.reference.n_points)
        self.sample_time_vector = range(self.reference.n_points)
        self.x_position_vector = range(self.reference.n_points)
        self.x_position_dot_vector = range(self.reference.n_points)
        self.y_position_vector = range(self.reference.n_points)
        self.y_position_dot_vector = range(self.reference.n_points)
        self.z_position_vector = range(self.reference.n_points)
        self.z_position_dot_vector = range(self.reference.n_points)
        self.x_ref_vector = range(self.reference.n_points)
        self.y_ref_vector = range(self.reference.n_points)
        self.value1 = range(self.reference.n_points)
        #self.current1_vector = range(self.reference.n_points)
        self.reference1 = range(self.reference.n_points)
        self.value2 = range(self.reference.n_points)
        #self.current2_vector = range(self.reference.n_points)
        self.reference2 = range(self.reference.n_points)
        self.timer_start()

    def experiment_init(self, save_file, smooth, track_parameters):

        self.smooth_flag = smooth
        #self.file_name = save_file
        self.experiment_reset()

        track_parameters['sample_time'] = self.sample_time

        self.reference.generate(**track_parameters)

        self.time_vector = range(self.reference.n_points)
        self.sample_time_vector = range(self.reference.n_points)
        self.x_position_vector = range(self.reference.n_points)
        self.x_position_dot_vector = range(self.reference.n_points)
        self.y_position_vector = range(self.reference.n_points)
        self.y_position_dot_vector = range(self.reference.n_points)
        self.z_position_vector = range(self.reference.n_points)
        self.z_position_dot_vector = range(self.reference.n_points)
        self.x_ref_vector = range(self.reference.n_points)
        self.y_ref_vector = range(self.reference.n_points)
        self.value1 = range(self.reference.n_points)
        #self.current1_vector = range(self.reference.n_points)
        self.reference1 = range(self.reference.n_points)
        self.value2 = range(self.reference.n_points)
        #self.current2_vector = range(self.reference.n_points)
        self.reference2 = range(self.reference.n_points)
        self.timer_start()

    def experiment_finish(self):
        self.arduino.set_speeds(0, 0)
        signal.setitimer(signal.ITIMER_REAL, 0, 0)
        #self.experiment_save()
        # print 'Done'
        # print self.ccc
        self.finished = True
    """
    def experiment_save(self):

        for i in range(0, self.count - 1):
            self.x_position_dot_vector[i] = (self.x_position_vector[i + 1] - self.x_position_vector[i]) / \
                                            (self.time_vector[i + 1] - self.time_vector[i])
            self.y_position_dot_vector[i] = (self.y_position_vector[i + 1] - self.y_position_vector[i]) / \
                                            (self.time_vector[i + 1] - self.time_vector[i])
            self.z_position_dot_vector[i] = (self.z_position_vector[i + 1] - self.z_position_vector[i]) / \
                                            (self.time_vector[i + 1] - self.time_vector[i])

        self.x_position_dot_vector[self.count - 1] = self.x_position_dot_vector[self.count - 2]
        self.y_position_dot_vector[self.count - 1] = self.y_position_dot_vector[self.count - 2]
        self.z_position_dot_vector[self.count - 1] = self.z_position_dot_vector[self.count - 2]

        # print("Sample time was not achieved: %4d times" % self.missed)
        # print("Sample time was achieved:     %4d times" % self.accurate)
        # print("Total:                        %4d times" % (self.missed + self.accurate))
        # print("Total Expected:               %4d times" % self.reference.n_points)
        # print("Worth sample time:            %12f ms" % (self.max_elapsed * 1000))
        # print("Desired sample time:          %12f ms" % (self.sample_time * 1000))

        save_file = open(self.file_name, 'w')

        save_file.write("close all;\r\n")
        save_file.write("clear all;\r\n")
        save_file.write("clc;\r\n\r\n")

        save_file.write("count=%f;\r\n" % self.count)
        save_file.write("radius=%f;\r\n" % self.radius)
        save_file.write("distance=%f;\r\n" % self.distance)
        save_file.write("constant_b=%f;\r\n" % self.constant_b)
        save_file.write("constant_k1=%f;\r\n" % self.constant_k1)
        save_file.write("constant_k2=%f;\r\n" % self.constant_k2)
        save_file.write("constant_ki=%f;\r\n" % self.constant_ki)
        save_file.write("constant_kd=%f;\r\n" % self.constant_kd)
        save_file.write("constant_kc=%f;\r\n" % self.constant_kc)

        for i in range(0, self.count):
            save_file.write("speed1(%d) = %f ;\r\n" % (i + 1, self.value1[i]))
            save_file.write("speed2(%d) = %f ;\r\n" % (i + 1, self.value2[i]))
            save_file.write("current1(%d) = %f ;\r\n" % (i + 1, self.current1_vector[i]))
            save_file.write("current2(%d) = %f ;\r\n" % (i + 1, self.current2_vector[i]))
            save_file.write("time(%d) = %f ;\r\n" % (i + 1, self.time_vector[i]))
            save_file.write("sample_time(%d) = %f ;\r\n" % (i + 1, self.sample_time_vector[i]))
            save_file.write("ref1(%d) = %f ;\r\n" % (i + 1, self.reference1[i]))
            save_file.write("ref2(%d) = %f ;\r\n" % (i + 1, self.reference2[i]))
            save_file.write("x(%d) = %f ;\r\n" % (i + 1, self.x_position_vector[i]))
            save_file.write("y(%d) = %f ;\r\n" % (i + 1, self.y_position_vector[i]))
            save_file.write("z(%d) = %f ;\r\n" % (i + 1, self.z_position_vector[i]))
            save_file.write("xd(%d) = %f ;\r\n" % (i + 1, self.reference.xd_vector[i]))
            save_file.write("yd(%d) = %f ;\r\n" % (i + 1, self.reference.yd_vector[i]))
            save_file.write("zd(%d) = %f ;\r\n" % (i + 1, self.reference.zd_vector[i]))
            save_file.write("dx(%d) = %f ;\r\n" % (i + 1, self.x_position_dot_vector[i]))
            save_file.write("dy(%d) = %f ;\r\n" % (i + 1, self.y_position_dot_vector[i]))
            save_file.write("dz(%d) = %f ;\r\n" % (i + 1, self.z_position_dot_vector[i]))
            save_file.write("dxd(%d) = %f ;\r\n" % (i + 1, self.reference.xd_dot_vector[i]))
            save_file.write("dyd(%d) = %f ;\r\n" % (i + 1, self.reference.yd_dot_vector[i]))
            save_file.write("dzd(%d) = %f ;\r\n" % (i + 1, self.reference.zd_dot_vector[i]))
            save_file.write("dxr(%d) = %f ;\r\n" % (i + 1, self.x_ref_vector[i]))
            save_file.write("dyr(%d) = %f ;\r\n" % (i + 1, self.y_ref_vector[i]))

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,x,time,xd) ;\r\n")
        save_file.write("title(\'X Position vs Time.\') ;\r\n")
        save_file.write("legend(\'X\',\'X - Reference\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'X Position (m)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,y,time,yd) ;\r\n")
        save_file.write("title(\'Y Position vs Time.\') ;\r\n")
        save_file.write("legend(\'Y\',\'Y - Reference\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Y Position (m)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,z,time,zd) ;\r\n")
        save_file.write("title(\'Orientation vs Time.\') ;\r\n")
        save_file.write("legend(\'Z\',\'Z - Reference\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Z Position (rad)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(x,y,xd,yd) ;\r\n")
        save_file.write("title(\'Y Position vs X Position (Path).\') ;\r\n")
        save_file.write("legend(\'Path\',\'Path - Reference\') ;\r\n")
        save_file.write("xlabel(\'X Position (m)') ;\r\n")
        save_file.write("ylabel(\'Y Position (m)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,speed1,time,ref1) ;\r\n")
        save_file.write("title(\'WL vs Time.\') ;\r\n")
        save_file.write("legend(\'WL\',\'WL - Reference\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Angular Speed (rad/s)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,speed2,time,ref2) ;\r\n")
        save_file.write("title(\'WR vs Time.\') ;\r\n")
        save_file.write("legend(\'WR\',\'WR - Reference\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Angular Speed (rad/s)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,current1,time,current2) ;\r\n")
        save_file.write("title(\'Currents vs Time.\') ;\r\n")
        save_file.write("legend(\'Left Motor\',\'Right Motor\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Current (A)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,dx,time,dxr,time,dxd) ;\r\n")
        save_file.write("title(\'X Speed vs Time.\') ;\r\n")
        save_file.write("legend(\'DX\',\'DXR - Reference\',\'DXD - Planning\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'X Speed (m/s)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,dy,time,dyr,time,dyd) ;\r\n")
        save_file.write("title(\'Y Speed vs Time.\') ;\r\n")
        save_file.write("legend(\'DY\',\'DYR - Reference\',\'DYD - Planning\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Y Speed (m/s)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(time,dz,time,dzd) ;\r\n")
        save_file.write("title(\'Z Speed vs Time.\') ;\r\n")
        save_file.write("legend(\'DZ\',\'DZD - Planning\') ;\r\n")
        save_file.write("xlabel(\'Time (s)') ;\r\n")
        save_file.write("ylabel(\'Z Speed (rad/s)') ;\r\n")
        save_file.write("grid on\r\n")

        save_file.write("\r\nfigure;\r\n")
        save_file.write("plot(sample_time*1000);\r\n")
        save_file.write("title('Sample Time.');\r\n")
        save_file.write("ylabel ( 'Sample Time (ms)' ) ;\r\n")
        save_file.write("xlabel ( 'Sample (k)' ) ;\r\n")
        save_file.write("grid on\r\n")

        save_file.close()
    """
    def experiment_reset(self):
        self.finished = False
        self.count = 0
        self.missed = 0
        self.accurate = 0
        self.max_elapsed = 0
        #self.u1k1 = 0
        #self.u2k1 = 0
        #self.e1k1 = 0
        #self.e1k2 = 0
        #self.e2k1 = 0
        #self.e2k2 = 0
        self.x_position = 0
        self.y_position = 0
        self.z_position = 0

        self.encoder1 = 0
        self.encoder2 = 0
        #self.current1 = 0
        #self.current2 = 0
        self.battery = 0

        self.prev_encoder1 = 0
        self.prev_encoder2 = 0
        self.prev_delta_encoder_1 = 0
        self.prev_delta_encoder_2 = 0

        self.safe_count = False
        self.safe_counter = 0

        self.arduino.reset_encoders()

    def timer_handler(self, signum, frame):

        if self.finished:
            self.experiment_finish()
            #self.to_list('experiment done')
            return

        encoder1, encoder2, battery = self.arduino.read_state()

        self.navigation(encoder1, encoder2)

        set_point1, set_point2 = self.tracking()

        self.arduino.set_speeds(set_point1, set_point2)

        self.count += 1

        if self.safe_count:
            self.safe_counter -= 1
            if self.safe_counter <= 0:
                self.finished = True

        if self.count >= self.reference.n_points:
            self.finished = True

        #self.to_list('position %f,%f,%f' % (self.globalPositionX, self.globalPositionY, self.globalPositionZ))

    def calculatePosition(self, encoder1, encoder2):
        delta_encoder_1 = encoder1 - self.prev_encoder1
        delta_encoder_2 = encoder2 - self.prev_encoder2

        self.encoder1 = encoder1
        self.encoder2 = encoder2

        if delta_encoder_1 > 130 or delta_encoder_1 < -130: #TODO: Calcular esos valores, tambien en enc 2
            delta_encoder_1 = self.prev_delta_encoder_1
            # self.ccc += 1
            self.encoder1 = 0
            self.encoder2 = 0
            self.arduino.reset_encoders()

        if delta_encoder_2 > 130 or delta_encoder_2 < -130:
            delta_encoder_2 = self.prev_delta_encoder_2
            # self.ccc += 1
            self.encoder1 = 0
            self.encoder2 = 0
            self.arduino.reset_encoders()

        if encoder1 > 2000000000 or encoder1 < -2000000000 or encoder2 > 2000000000 or encoder2 < -2000000000:
            self.encoder1 = 0
            self.encoder2 = 0
            self.arduino.reset_encoders()

        self.prev_encoder1 = self.encoder1
        self.prev_encoder2 = self.encoder2

        self.prev_delta_encoder_1 = delta_encoder_1
        self.prev_delta_encoder_2 = delta_encoder_2

        dfr = delta_encoder_2 * 2 * math.pi / ENCODER_STEPS
        dfl = delta_encoder_1 * 2 * math.pi / ENCODER_STEPS

        ds = (dfr + dfl) * self.radius / 2
        dz = (dfr - dfl) * self.radius / self.distance

        self.x_position += ds * math.cos(self.z_position + dz / 2)
        self.y_position += ds * math.sin(self.z_position + dz / 2)
        self.z_position += dz

        self.globalPositionX += ds * math.cos(self.globalPositionZ + dz / 2)
        self.globalPositionY += ds * math.sin(self.globalPositionZ + dz / 2)
        self.globalPositionZ += dz

        return delta_encoder_1, delta_encoder_2

    def navigation(self, encoder1, encoder2):
        delta_encoder_1, delta_encoder_2 = self.calculatePosition(encoder1, encoder2)

        self.x_position_vector[self.count] = self.x_position
        self.y_position_vector[self.count] = self.y_position
        self.z_position_vector[self.count] = self.z_position

        return delta_encoder_1, delta_encoder_2

    def tracking(self):
        xd = self.reference.xd_vector[self.count]
        xd_dot = self.reference.xd_dot_vector[self.count]
        yd = self.reference.yd_vector[self.count]
        yd_dot = self.reference.yd_dot_vector[self.count]

        zd = self.reference.zd_vector[self.count]
        zd_dot = self.reference.zd_dot_vector[self.count]

        # vd = math.sqrt(xd_dot ** 2 + yd_dot ** 2)
        # wd = zd_dot
        #
        # constant_chi = 1
        # constant_a = 1
        #
        # k1 = 2 * constant_chi * constant_a
        # if vd:
        #     k2 = (constant_a ** 2 - wd ** 2) / vd
        # else:
        #     k2 = 0.0
        # k3 = k1
        #
        # e1 = math.cos(self.z_position) * (xd - self.x_position) + math.sin(self.z_position) * (yd - self.y_position)
        # e2 = -math.sin(self.z_position) * (xd - self.x_position) + math.cos(self.z_position) * (yd - self.y_position)
        # e3 = zd - self.z_position
        #
        # u1 = -k1 * e1
        # # u2 = -k2 * e2 - k3 * e3
        # if e3:
        #     u2 = -k2 * vd * math.sin(e3) / e3 * e2 - k3 * e3
        # else:
        #     u2 = -k2 * vd * e2 - k3 * e3
        #
        # the_v = vd * math.cos(e3) - u1
        # the_omega = wd - u2

        y1 = self.x_position + self.constant_b * math.cos(self.z_position)
        y2 = self.y_position + self.constant_b * math.sin(self.z_position)

        if self.smooth_flag:
            y1d = xd
            y2d = yd
            y2d_dot = yd_dot
            y1d_dot = xd_dot
        else:
            y1d = xd + self.constant_b * math.cos(zd)
            y2d = yd + self.constant_b * math.sin(zd)

            y2d_dot = yd_dot + self.constant_b * math.cos(zd) * zd_dot
            y1d_dot = xd_dot - self.constant_b * math.sin(zd) * zd_dot

        u2 = y2d_dot + self.constant_k2 * (y2d - y2)
        u1 = y1d_dot + self.constant_k1 * (y1d - y1)

        self.x_ref_vector[self.count] = u1
        self.y_ref_vector[self.count] = u2

        the_v = math.cos(self.z_position) * u1 + u2 * math.sin(self.z_position)
        the_omega = u1 * (- math.sin(self.z_position) / self.constant_b) + u2 * math.cos(
            self.z_position) / self.constant_b

        set_point2 = the_v / self.radius + the_omega * self.distance / 2 / self.radius
        set_point1 = the_v / self.radius - the_omega * self.distance / 2 / self.radius

        return set_point1, set_point2

    def process_time(self):
        now = time.time()
        elapsed = now - self.prev

        if self.count == 0:
            self.time_vector[0] = 0
        else:
            self.time_vector[self.count] = self.time_vector[self.count - 1] + elapsed
        self.sample_time_vector[self.count] = elapsed

        self.prev = now
        self.max_elapsed = elapsed if self.max_elapsed < elapsed else self.max_elapsed
        if elapsed > self.sample_time:
            self.missed += 1
        else:
            self.accurate += 1

        return elapsed
    """
    def speeds_regulation(self, set_point1, set_point2, delta_encoder_1, delta_encoder_2, elapsed, current1, current2, battery):
        self.current1 = current1
        self.current2 = current2
        self.battery = battery

        steps_per_sec1 = delta_encoder_1 / elapsed
        angular_speed1 = steps_per_sec1 * 2 * math.pi / ENCODER_STEPS

        self.value1[self.count] = angular_speed1
        self.current1_vector[self.count] = current1
        self.reference1[self.count] = set_point1

        steps_per_sec2 = delta_encoder_2 / elapsed
        angular_speed2 = steps_per_sec2 * 2 * math.pi / ENCODER_STEPS

        self.value2[self.count] = angular_speed2
        self.current2_vector[self.count] = current2
        self.reference2[self.count] = set_point2

        e1k = set_point1 - angular_speed1

        if self.constant_ki == 0:
            uuu1 = e1k * self.constant_kc
        else:
            uuu1 = e1k * self.constant_kc + (self.constant_ki - self.constant_kc) * self.e1k1 + self.u1k1

        if self.constant_kd != 0:
            uuu1 = self.constant_kc * (e1k - self.e1k1) + self.constant_ki * e1k + self.u1k1 + self.constant_kd * (
                e1k - 2 * self.e1k1 + self.e1k2)

        if uuu1 > 127.0:
            uuu1 = 127.0
        if uuu1 < - 128.0:
            uuu1 = - 128.0

        self.u1k1 = uuu1
        self.e1k2 = self.e1k1
        self.e1k1 = e1k

        um1 = int(uuu1)

        e2k = (set_point2 - angular_speed2)

        if self.constant_ki == 0:
            uuu2 = e2k * self.constant_kc
        else:
            uuu2 = e2k * self.constant_kc + (self.constant_ki - self.constant_kc) * self.e2k1 + self.u2k1

        if self.constant_kd != 0:
            uuu2 = self.constant_kc * (e2k - self.e2k1) + self.constant_ki * e2k + self.u2k1 + self.constant_kd * (
                e2k - 2 * self.e2k1 + self.e2k2)

        if uuu2 > 127.0:
            uuu2 = 127.0
        if uuu2 < - 128.0:
            uuu2 = - 128.0

        self.u2k1 = uuu2
        self.e2k2 = self.e2k1
        self.e2k1 = e2k

        um2 = int(uuu2)

        return um1, um2
    """
    def forward(self, value):
        if value:
            if self.finished:
                self.smooth_flag = True
                self.experiment_reset()

                self.safe_count = True
                self.safe_counter = 20

                track_parameters = {'x_planning': [0, value * 100.0],
                                    'y_planning': [0, 0],
                                    't_planning': [0, 100.0],
                                    'sample_time': self.sample_time}

                self.reference.generate_backward(**track_parameters)

                self.time_vector = range(self.reference.n_points)
                self.sample_time_vector = range(self.reference.n_points)
                self.x_position_vector = range(self.reference.n_points)
                self.x_position_dot_vector = range(self.reference.n_points)
                self.y_position_vector = range(self.reference.n_points)
                self.y_position_dot_vector = range(self.reference.n_points)
                self.z_position_vector = range(self.reference.n_points)
                self.z_position_dot_vector = range(self.reference.n_points)
                self.x_ref_vector = range(self.reference.n_points)
                self.y_ref_vector = range(self.reference.n_points)
                self.value1 = range(self.reference.n_points)
                ##self.current1_vector = range(self.reference.n_points)
                self.reference1 = range(self.reference.n_points)
                self.value2 = range(self.reference.n_points)
                ##self.current2_vector = range(self.reference.n_points)
                self.reference2 = range(self.reference.n_points)
                self.timer_start()
            else:
                # self.safe_count = True
                self.safe_counter = 20
        else:
            self.finished = True

    def rightward(self, value):
        if value:
            if self.finished:
                self.smooth_flag = True
                self.experiment_reset()

                self.safe_count = True
                self.safe_counter = 20

                track_parameters = {'z_planning': [0, 100.0 * value],
                                    't_planning': [0, 100.0],
                                    'sample_time': self.sample_time}

                self.reference.generate_rotation(**track_parameters)

                self.time_vector = range(self.reference.n_points)
                self.sample_time_vector = range(self.reference.n_points)
                self.x_position_vector = range(self.reference.n_points)
                self.x_position_dot_vector = range(self.reference.n_points)
                self.y_position_vector = range(self.reference.n_points)
                self.y_position_dot_vector = range(self.reference.n_points)
                self.z_position_vector = range(self.reference.n_points)
                self.z_position_dot_vector = range(self.reference.n_points)
                self.x_ref_vector = range(self.reference.n_points)
                self.y_ref_vector = range(self.reference.n_points)
                self.value1 = range(self.reference.n_points)
                #self.current1_vector = range(self.reference.n_points)
                self.reference1 = range(self.reference.n_points)
                self.value2 = range(self.reference.n_points)
                #self.current2_vector = range(self.reference.n_points)
                self.reference2 = range(self.reference.n_points)
                self.timer_start()
            else:
                # self.safe_count = True
                self.safe_counter = 20
        else:
            self.finished = True

    def backward(self, value):
        if value:
            if self.finished:
                self.smooth_flag = True
                self.experiment_reset()

                self.safe_count = True
                self.safe_counter = 20

                track_parameters = {'x_planning': [0, -value * 100.0],
                                    'y_planning': [0, 0],
                                    't_planning': [0, 100.0],
                                    'sample_time': self.sample_time}

                self.reference.generate_backward(**track_parameters)

                self.time_vector = range(self.reference.n_points)
                self.sample_time_vector = range(self.reference.n_points)
                self.x_position_vector = range(self.reference.n_points)
                self.x_position_dot_vector = range(self.reference.n_points)
                self.y_position_vector = range(self.reference.n_points)
                self.y_position_dot_vector = range(self.reference.n_points)
                self.z_position_vector = range(self.reference.n_points)
                self.z_position_dot_vector = range(self.reference.n_points)
                self.x_ref_vector = range(self.reference.n_points)
                self.y_ref_vector = range(self.reference.n_points)
                self.value1 = range(self.reference.n_points)
                #self.current1_vector = range(self.reference.n_points)
                self.reference1 = range(self.reference.n_points)
                self.value2 = range(self.reference.n_points)
                #self.current2_vector = range(self.reference.n_points)
                self.reference2 = range(self.reference.n_points)
                self.timer_start()
            else:
                # self.safe_count = True
                self.safe_counter = 20
        else:
            self.finished = True

    def continuous(self, parameter, value):
        if parameter == 'forward':
            self.forward(value)
        elif parameter == 'backward':
            self.backward(value)
        elif parameter == 'right':
            self.rightward(value)
        elif parameter == 'left':
            if value:
                pass
            else:
                self.finished = True

    def timer_start(self):
        self.prev = time.time()
        signal.setitimer(signal.ITIMER_REAL, self.sample_time, self.sample_time)

    def timer_init(self):
        signal.signal(signal.SIGALRM, self.timer_handler)
        signal.setitimer(signal.ITIMER_REAL, 0, 0)

    def rotate_inplace(self, speed):
        self.arduino.set_speeds(speed, -speed)

    def stop(self):
        self.arduino.set_speeds(0,0)

    # WASD movement
    # TODO: Update positions

    def move(self):
        x, y = 0, 0
        sleep_time = .01
        while True:
            dx, dy = 0, 0

            if 87 in self.keys:  # w
                dy += 8
            if 65 in self.keys:  # a
                dx += -8
            if 83 in self.keys:  # s
                dy += -8
            if 68 in self.keys:  # d
                dx += 8

            y = (y + dy) / 2.0
            x = (x + dx) / 2.0

            right, left = self.wasd_velocities(x, y)
            if (right or left):
                encoder1, encoder2, _ = self.arduino.read_state()
                self.calculatePosition(encoder1, encoder2)
            else:
                continue

            self.arduino.set_speeds(right, left)
            #print("position %f,%f" %(self.globalPositionX, self.globalPositionY))
            #self.to_list('position %f,%f,%f' % (self.globalPositionX, self.globalPositionY, self.globalPositionZ))
            time.sleep(sleep_time)

    def wasd_velocities(self, x, y):
        left = right = y

        # ratio = abs(x/8.0)  #Ratio to control robot's speed. Correct for faster movement
        ratio = abs(x/16.0)

        if x > 0:
            right *= (1-ratio)
        elif x < 0:
            left *= (1-ratio)

        return right, left
