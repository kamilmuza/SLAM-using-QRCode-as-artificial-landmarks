from pprint import pprint

__author__ = 'Geist'
import math

import numpy as np
import matplotlib.pyplot as plt

import tools
import drawing


#import and configure logging
import logging
logging.basicConfig(filename='example.log',level=logging.DEBUG,format='%(asctime)s - %(levelname)s - %(message)s')
logging.debug('This message should go to the log file')

#finish configure logging

# import scipy as sp

# TODO: Add exception handling for numpy operations and input of data...
# TODO: Checking of proper input
# TODO: Manage Landmarks
# TODO: Make simulator
# TODO: Make plotting facilities for debugging...
# TODO: Check difference between using pinv (from scipy), pinv2  or inv (which is faster?)
# TODO: Check utilities in frameworks package in Scipy or Numpy (or port from Matlab)


# Graph Slam Offline Implementation
class GraphSlam:
    def __init__(self, dim=2, landmarks=10, pose=(0, 0), poses=10):
        self.nLandmarks = landmarks  # Number of landmarks
        self.nPoses = poses  # Number of Poses to keep track of
        self.currentPose = 0  # The current Pose
        self.dim = dim
        n = (landmarks + poses) * dim
        self.Omega = np.zeros((n, n), dtype=np.float)
        self.Xi = np.zeros(n, dtype=np.float)
        # Initial Conditions
        if self.dim == 1:
            pose = np.asarray(pose).reshape(1)
        for i in range(dim):
            self.Omega[i, i] = 1
            self.Xi[i] = pose[i]
        self.update_map = True  # If we have to calculate the map...
        self.map = None

    def get_map(self):
        if self.update_map:
            inv_omega = (np.linalg.pinv(self.Omega))
            self.map = inv_omega.dot(self.Xi)
            self.update_map = False
        return self.map

    def add_landmarks(self, numLand=1):
        self.update_map = True
        # Grow array to incorporate new Landmarks
        a = np.zeros((numLand * self.dim, 1))
        self.Omega = np.insert(self.Omega, self.Omega.shape[0], a, axis=0)
        self.Omega = np.insert(self.Omega, self.Omega.shape[1], a, axis=1)
        a.shape = (numLand * self.dim)
        self.Xi = np.insert(self.Xi, self.Xi.shape[0], a, axis=0)
        # Update number of landmarks
        self.nLandmarks += numLand

    def add_poses(self, numPos=1, atPose=False):
        self.update_map = True
        # Grow array to incorporate new poses
        a = np.zeros((numPos * self.dim, 1))
        # Pose index to insert new Poses
        if not atPose:
            pose = self.currentPose + 1
        else:
            pose = atPose
        index = pose * self.dim
        self.Omega = np.insert(self.Omega, index, a, axis=0)
        self.Omega = np.insert(self.Omega, index, a, axis=1)
        a.shape = (numPos * self.dim)
        self.Xi = np.insert(self.Xi, index, a, axis=0)
        # Update number of poses
        self.nPoses += numPos

    def update_landmark(self, numLand, pose, noise):
        if self.dim == 1:
            pose = np.asarray(pose).reshape(1)
        self.update_map = True
        index1 = self.currentPose * self.dim
        index2 = (self.nPoses + numLand - 1) * self.dim
        add = np.array([[1, -1], [-1, 1]]) / noise
        for i in range(self.dim):
            sum_xi = np.array([-pose[i], pose[i]]) / noise
            # Update dim i
            self.Omega[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            self.Xi[index1:index2 + 1:index2 - index1] += sum_xi
            index1 += 1
            index2 += 1
            # # Update Y
            # index1 += 1
            # index2 += 1
            # self.Omega[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            # self.Xi[index1:index2 + 1:index2 - index1] += np.array([-y, y]) / noise

    def update_landmark_from_pose(self, numPose, numLand, pose, noise):
        current_pose = self.currentPose
        self.currentPose = numPose
        self.update_landmark(numLand, pose, noise)
        self.currentPose = current_pose

    def update_pose(self, pose, noise):
        if self.dim == 1:
            pose = np.asarray(pose).reshape(1)
        self.update_map = True
        index1 = self.currentPose * self.dim
        index2 = (self.currentPose + 1) * self.dim
        add = np.array([[1, -1], [-1, 1]]) / noise
        for i in range(self.dim):
            sum_xi = np.array([-pose[i], pose[i]]) / noise
            # Update dim i
            self.Omega[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            self.Xi[index1:index2 + 1:index2 - index1] += sum_xi

            index1 += 1
            index2 += 1
            # # Update X
            # self.Omega[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            # self.Xi[index1:index2 + 1:index2 - index1] += [-x, x]
            # # Update Y
            # index1 += 1
            # index2 += 1
            # self.Omega[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            # self.Xi[index1:index2 + 1:index2 - index1] += [-y, y]
        self.currentPose += 1


# Graph Slam Online Implementation
class GraphSlamOnline(GraphSlam):
    def __init__(self, dim=2, landmarks=10, pose=(0, 0)):
        GraphSlam.__init__(self, dim, landmarks, pose)
        # self.nLandmarks = landmarks
        # self.dim = dim
        # self.currentPose = 0
        self.nPoses = 1  # We retain only one pose, not the history...
        n = (landmarks + self.nPoses) * dim
        self.Omega_big = np.zeros((n + dim, n + dim), dtype=np.float)
        self.Xi_big = np.zeros(n + dim, dtype=np.float)
        self.Omega = self.Omega_big[dim:, dim:]
        self.Xi = self.Xi_big[dim:]
        if self.dim == 1:
            pose = np.asarray(pose).reshape(1)
        for i in range(dim):
            self.Omega[i, i] = 1
            self.Xi[i] = pose[i]
        # self.update_map = True  # If we have to calculate the map...
        # self.map = None
        self.A = self.Omega_big[0:dim, dim:]
        self.B = self.Omega_big[0:dim, 0:dim]
        self.C = self.Xi_big[0:dim]

    def update_pose(self, pose, noise):
        if self.dim == 1:
            pose = np.asarray(pose).reshape(1)
        # Get A, B, C, Omega' and Xi' and calculate the new Omega and Xi
        # We have only the matrices OmegaBig and XiBig, all the other matrices are views so
        # we don't need to create extra matrices. We simulate here the enlargement of the matrix
        # for the new pose...
        self.A[0:self.dim, self.dim:] = self.Omega[0:self.dim, self.dim:]
        self.A[0:self.dim, 0:self.dim] = 0
        self.Omega[0:self.dim, self.dim:] = 0
        # A transpose
        self.Omega[self.dim:, 0:self.dim] = 0
        self.B[:, :] = self.Omega[0:self.dim, 0:self.dim]
        self.Omega[0:self.dim, 0:self.dim] = 0
        self.C[:] = self.Xi[0:self.dim]
        self.Xi[0:self.dim] = 0
        # Introduce the new values
        self.update_map = True
        index1 = 0
        index2 = self.dim
        add = np.array([[1, -1], [-1, 1]]) / noise
        for i in range(self.dim):
            sum_xi = np.array([-pose[i], pose[i]]) / noise
            # Update dim i
            self.Omega_big[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            self.Xi_big[index1:index2 + 1:index2 - index1] += sum_xi
            index1 += 1
            index2 += 1
            # # Update X
            # self.Omega_big[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            # self.Xi_big[index1:index2 + 1:index2 - index1] += [-x, x]
            # # Update Y
            # index1 += 1
            # index2 += 1
            # self.Omega_big[index1:index2 + 1:index2 - index1, index1:index2 + 1:index2 - index1] += add
            # self.Xi_big[index1:index2 + 1:index2 - index1] += [-y, y]
        # And calculate the new Omega and Xi
        inv_b = np.linalg.inv(self.B)
        self.Omega[:, :] = self.Omega - self.A.transpose().dot(inv_b).dot(self.A)
        self.Xi[:] = self.Xi - self.A.transpose().dot(inv_b).dot(self.C)

    def add_poses(self, numPos=1, atPose=False):
        pass

    def update_landmark_from_pose(self, numPose, numLand, pose, noise):
        pass


class EKFSlam:
    """
    EKF SLAM implementation
        Assumptions:
            Robot moves in the plane
            Known data association
            Range-Bearing sensor
            Point Landmarks
            Robot moves in 2d
        Cycle:
            1. State prediction
            2. Measurement prediction
            3. Measurement
            4. Data association
            5. Update
    """
    # TODO: Add noise matrices Rt and Qt and find how to calculate it...
    # TODO: Handle correlation of data
    # TODO: Handle landmark initialization automatically... (keepeing a list of landmarks observed)

    def __init__(self, landmarks=10, pose=(0, 0, 0), model='o'):
        # Motion model is odometry base or velocity base
        if model not in ('o', 'v'):
            raise Exception('Wrong motion model')
        if model == 'o':
            self.motion_model = self.predict_odometry_model
        else:
            self.motion_model = self.predict_velocity_model
        # if dim > 3 or dim < 1:
        # raise Exception('Wrong number of dimension, must be between 1 and 3')
        dim = 2
        self.nLandmarks = landmarks  # Number of landmarks
        self.nInitLandmarks = 0  # Number of Landmarks that have already been observe
        self.dim = dim
        # 2d SLAM has a dimension of 2*N_landmark + 3 (x,y,theta of the robot)
        # 3d SLAM (not checked yet but...) must have 3*N_landmark + 6 (x,y,z + roll,pitch,yaw)
        # t = int(dim * (dim + 1) / 2)
        # self.n = landmarks * dim + t
        self.n = landmarks * dim + 3
        self.Sigma = np.eye(self.n, self.n, dtype=np.float)  # Covariance matrix
        self.Xt = np.zeros(self.n, dtype=np.float)  # Mean vector
        # Initial Conditions
        # if self.dim == 1:
        # pose = np.asarray(pose).reshape(1)

        # Initialize Pose and Covariance matrix Sigma
        # self.Sigma = self.Sigma * np.inf
        self.Sigma = self.Sigma * 10000
        for i in range(3):
            self.Xt[i] = pose[i]
            self.Sigma[i, i] = 0
        # Create windows for easier access
        self.Pose = self.Xt[:3]
        self.Landmarks = self.Xt[3:]
        self.SigmaXX = self.Sigma[:3, :3]
        self.SigmaXM = self.Sigma[:3, 3:]
        self.SigmaMX = self.Sigma[3:, :3]
        self.SigmaMM = self.Sigma[3:, 3:]
        # Array to save the believe of the landmarks
        # BYME: Really necessary?
        self.z_t = np.ndarray(self.Landmarks.shape, dtype=self.Landmarks.dtype)

    def get_pose(self):
        return self.Pose

    def get_pose_covariance(self):
        # pprint(self.Sigma[0:3, 0:3])
        return self.Sigma[0:3, 0:3]

    def get_landmark(self, n):
        if n > self.nLandmarks or n < 1:
            raise ValueError('There isn\'t a Landmark with number:%d' % n)
        n -= 1
        return self.Landmarks[self.dim * n:self.dim * n + self.dim]

    def get_landmark_covariance(self, n):
        if n > self.nLandmarks or n < 1:
            raise ValueError('There isn\'t a Landmark with that number')
        n -= 1
        start = 3 + self.dim * n
        end = start + self.dim
        return self.Sigma[start:end, start:end]

    def get_landmarks(self):
        return self.Landmarks

    def init_landmark(self, rang, bearing, n=None):
        """
        Given a range and bearing, initialize the pose of a landmark relative to the pose of the robot
        The pose in this case is
        self.Pose[0] = Xt
        self.Pose[1] = Yt
        self.Pose[2] = Theta

        @return:
        """
        if self.nInitLandmarks == self.nLandmarks:
            self.add_landmarks()
        if n is None:
            n = self.nInitLandmarks
        self.nInitLandmarks += 1
        self.Landmarks[n * self.dim] = self.Pose[0] + rang * math.cos(bearing + self.Pose[2])
        self.Landmarks[n * self.dim + 1] = self.Pose[1] + rang * math.sin(bearing + self.Pose[2])

    def get_expected_observations(self):
        """


        @return:
        """
        delta_x = self.Landmarks[::self.dim] - self.Pose[0]
        delta_y = self.Landmarks[1::self.dim] - self.Pose[1]
        q = np.sqrt(np.square(delta_x) + np.square(delta_y))
        theta = np.arctan2(delta_y, delta_x) - self.Pose[2]
        self.z_t[::self.dim] = q
        self.z_t[1::self.dim] = theta
        return self.z_t

    def get_expected_landmark_observation(self, n_landmark):
        """

        @param n_landmark:
        @return: list
        """
        n = n_landmark - 1
        delta_x = self.Landmarks[2 * n] - self.Pose[0]
        delta_y = self.Landmarks[2 * n + 1] - self.Pose[1]
        q2 = np.square(delta_x) + np.square(delta_y)
        # print n_landmark,delta_x,delta_y

        q = np.sqrt(q2)
        theta = np.arctan2(delta_y, delta_x) - self.Pose[2]
        z = np.array([q, theta])
        z.shape = (2, 1)
        h = np.array([
            [-q * delta_x, - q * delta_y, 0, q * delta_x, q * delta_y],
            [delta_y, - delta_x, - q2, - delta_y, delta_x]
        ]) / q2
        # return [delta, q2, q, z]
        return [h, z]

    def add_landmarks(self, numLand=1):
        pass

    def predict(self, u, noise=0.1):
        # Predict the new mean (depends on the model)
        # Get the Jacobian of the motion
        # Update the covariance matrix
        gx = self.motion_model(u)
        # print "Gx"
        # print gx
        # TODO: check why the views have problems, meanwhile used them directly
        # self.SigmaXX = gx.dot(self.SigmaXX.dot(gx.transpose()))
        self.Sigma[:3, :3] = gx.dot(self.Sigma[:3, :3].dot(gx.transpose()))
        # self.SigmaXM = gx.dot(self.SigmaXM)
        self.Sigma[:3, 3:] = gx.dot(self.Sigma[:3, 3:])
        # self.SigmaMX = self.SigmaMX.dot(gx.transpose())
        # self.SigmaMX = self.SigmaXM.transpose()
        self.Sigma[3:, :3] = self.Sigma[:3, 3:].transpose()
        # Add Rx (noise)
        r_x = np.eye(3) * noise
        r_x[2, 2] /= 10
        # self.SigmaXX += r_x
        self.Sigma[:3, :3] = self.Sigma[:3, :3] + r_x
        # Normalize angle...
        self.Pose[2] = tools.normalize_angle(self.Pose[2])

    def correct(self, observations, noise=0.1):
        """
        Correction step.
        Remember to initialize landmarks outside...!!!
        @param observations: list in the form [[index,range,bearing],...]
        @param noise:
        @rtype : none
        """
        # Update for every observed landmark
        # Build Qr, noise for the measurement. Assume diagonal matrix for Q
        Q = np.eye(2) * noise
        for zi in observations:
            # zi is current observation
            n = zi[0]
            z = np.array(zi[1:])
            z.shape = (2, 1)
            # Get Jacobian and expected observation
            (h, z_p) = self.get_expected_landmark_observation(n)
            # print "HiLow:"
            # print h
            # Build Fx to map low dimensional h to high dimensional H
            fx = np.zeros((5, self.n), dtype=int)
            fx[:3, :3] = np.eye(3)
            i = 3 + (n - 1) * 2
            # fx[3:5,i:i+2] = np.eye(2)
            fx[3, i] = 1
            fx[4, i + 1] = 1
            H = h.dot(fx)
            # P:Sigma
            # Calculate Kalman gain
            # K = P*H'*S where S = inv(HPH' + Q)
            S = H.dot(self.Sigma.dot(H.transpose())) + Q
            K = self.Sigma.dot(H.transpose().dot(np.linalg.pinv(S)))
            # Correct the new mean (depends on the observations)
            # Xt = Xt-1 + K*(z-z')
            # Remember to normalize the bearings...
            dz = tools.normalize_bearings(z - z_p)
            # Kt = K.dot(z - z_p)
            Kt = K.dot(dz)
            Kt.shape = self.Xt.shape
            # print Kt + self.Xt
            # self.Xt += K.dot(z - z_p)
            self.Xt += Kt
            # print self.Pose
            # print Kt.shape
            # exit()
            # Update the covariance matrix
            # P = (I- K*H)*P
            self.Sigma = (np.eye(self.n) - K.dot(H)).dot(self.Sigma)
            # And normalize the pose angle...
            self.Pose[2] = tools.normalize_angle(self.Pose[2])
            # self.print_pose()

    def correct_batch(self, observations, noise=0.1):
        # TODO:According to the dimensionality, one is faster than the other, check for a way to change accordingly...
        """
        The main difference between this and the previous method is the computational complexity.
         In one, you do i(number of observations) inversions of a 2x2 matrix,
         and update the covariance matrix for i times.
         In the other, you do one inversions of a 2ix2i matrix,
         and update the covariance matrix one times.
        @param observations: list in the form [[index,range,bearing],...]
        @param noise: noise in the observation. Used to build Q (assume diagonal).
        @rtype : none
        """
        n_land = len(observations) * 2
        # Update for observed landmarks in batch...
        # Build Qr, noise for the measurement. Assume diagonal matrix for Q
        Q = np.eye(n_land) * noise
        # Error y = z - z_p
        y = np.zeros((n_land))
        # Build high dimensional H
        H = np.zeros((n_land, self.n))
        index = 0
        for zi in observations:
            # Build low dimensional h
            n = zi[0]
            z = np.array(zi[1:])
            z.shape = (2, 1)
            (h, z_p) = self.get_expected_landmark_observation(n)
            # Build Fx to map low dimensional h to high dimensional H of the observation
            fx = np.zeros((5, self.n), dtype=int)
            fx[:3, :3] = np.eye(3)
            i = 3 + (n - 1) * 2
            # fx[3:5,i:i+2] = np.eye(2)
            fx[3, i] = 1
            fx[4, i + 1] = 1
            Hz = h.dot(fx)
            # Stack them together
            H[index:index + 2, :] = Hz
            y[index:index + 2] = z - z_p
            index += 2

        # P:Sigma
        # K = P*H'* inv(HPH' + Q)
        S = H.dot(self.Sigma.dot(H.transpose())) + Q
        # Calculate Kalman gain
        K = self.Sigma.dot(H.transpose().dot(np.linalg.pinv(S)))
        # correct the new mean (depends on the observations)
        # Xt = Xt-1 + K*(z-z')
        self.Xt += K.dot(y)
        # Update the covariance matrix
        # P = (I- K*H)*P
        self.Sigma = (np.eye(self.n) - K.dot(H)).dot(self.Sigma)
        # self.Pose[2] = tools.normalize_angle(self.Pose[2])

    def predict_velocity_model(self, u):
        """
        Uses velocity model to update the mean.
        U = [ V, W, dt]
        x = x - v/w * sin(theta) + v/w * sin(theta + w*dt)
        y = y + v/w * cos(theta) - v/w * cos(theta + w*dt)
        theta = theta + w*dt
        Returns Jacobian of motion Gx
        """
        # Velocity model linear and angular velocity
        v = u[0]
        w = u[1]
        dt = u[2]
        th = self.Pose[2]
        n_th = w * dt
        vw = v / w
        t1 = - vw * math.sin(th) + vw * math.sin(th + n_th)
        t2 = vw * math.cos(th) - vw * math.cos(th + n_th)
        # Update pose
        self.Pose[0] += t1
        self.Pose[1] += t2
        self.Pose[2] += n_th
        # Get Jacobian Gx
        gx = np.eye(3)
        gx[0, 2] = -t2
        gx[1, 2] = t1
        return gx

    def predict_odometry_model(self, u):
        """
        Uses odometry model to update the mean.
        U = [ r1, t, r2]
        x = x + t*cos(theta + r1)
        y = y + t*sin(theta + r1)
        theta = theta + r1+r2
        Returns Jacobian of motion Gx
        """
        # Odometry model rotations and translations
        r1 = u[0]
        t = u[1]
        r2 = u[2]

        t1 = t * math.cos(self.Pose[2] + r1)
        t2 = t * math.sin(self.Pose[2] + r1)

        # Update Pose
        self.Pose[0] += t1
        self.Pose[1] += t2
        self.Pose[2] += r1 + r2
        # Get Jacobian Gx
        gx = np.eye(3)
        gx[0, 2] = -t2
        gx[1, 2] = t1
        # Return small Jacobian Gx or full Jacobian G ???
        return gx

    def print_pose(self):
        print 'X:%+04.4f\tY:%+04.4f\tTheta:%+04.4f' % (self.Pose[0], self.Pose[1], self.Pose[2])

    def print_covariance(self):
        pprint(self.Sigma)


class UKFSlam:
    """
    UKF SLAM implementation
    Unscented version of the Kalman Filter...
    """

    def __init__(self, dim=2, landmarks=10, pose=(0, 0)):
        pass

    def get_pose(self):
        pass

    def get_landmark(self, n):
        pass

    def get_landmarks(self):
        pass

    def add_landmarks(self, numLand=1):
        pass

    def predict(self, numLand, pose, noise):
        pass

    def correct(self, numPose, numLand, pose, noise):
        pass


if __name__ == "__main__":
    data = tools.read_data('data/sensor.dat')
    world = tools.read_world('data/world.dat')
    f = plt.Figure(figsize=(5, 5), dpi=100, )
    f.set_visible(False)
    plt.axes()

    # Accommodate landmarks to plot...
    lx = []
    ly = []
    for landmark in world:
        lx.append(landmark['x'])
        ly.append(landmark['y'])
    landmarks = [lx, ly]

    print "Beginning EKFSlam Test"
    # Keep track of the observed landmarks
    observed_landmarks = []
    slam = EKFSlam(7)

    for (step, reading) in enumerate(data):
        logging.info('Step number:'+str(step))
        # if step == 10:
        # break
        r1 = tools.normalize_angle(reading['odometry']['r1'])
        r2 = tools.normalize_angle(reading['odometry']['r2'])
        odometry = [r1, reading['odometry']['t'], r2]
        slam.predict(odometry)
        # print "Mu and Sigma after Prediction"
        # print "Mu:"
        # print slam.Xt
        # print "Sigma:"
        # print slam.Sigma
        # print "\t\tr1: "+str(odometry[0])+" t: "+str(odometry[1])+" r2 :"+str(odometry[2])
        # Correct with each measurement
        sensor_reading = []
        for sensor in reading['sensor']:
            # sensor_reading = [sensor['id'], sensor['range'], sensor['bearing']]
            if sensor['id'] != 0:
                logging.info('Seeing landmark id:'+str(sensor['id']))
                bearing = tools.normalize_angle(sensor['bearing'])
                logging.warning('range:%.5f bearing:%.5f'%(sensor['range'], bearing))
                sensor_reading.append([sensor['id'], sensor['range'], bearing])
                if sensor['id'] not in observed_landmarks:
                    observed_landmarks.append(sensor['id'])
                    slam.init_landmark(sensor['range'], bearing, sensor['id'] - 1)
                    logging.info('Loggin landmark id:%d'%sensor['id'])
                    logging.warning('x:%.5f y:%.5f'%(slam.get_landmark(6)[0],slam.get_landmark(6)[1]))
                # print "Sensor Reading"
                # print "\t\tid: "+str(sensor['id'])+" range: "+str(sensor['range'])+" bearing: "+str(sensor['bearing'])
        #if len(sensor_reading) != 0:
            #print 'Correcting in step %d'%step
        slam.correct(sensor_reading)
        # print "Mu and Sigma after Correction"
        # print "Mu:"
        # print slam.Xt
        # print "Sigma:"
        # print slam.Sigma
        # print "Current pose at time step: " + str(step)
        print "step:" + str(step + 1) + " ",
        slam.print_pose()
        # pprint(slam.Sigma)
        drawing.draw_state(slam, landmarks, observed_landmarks, step + 1, sensor_reading)
    print "\n\nObserved landmarks:"
    print observed_landmarks
    print "\n\nWorld landmarks:"
    print world
    print slam.get_landmarks()
    print "\n\nSigma Robot:"
    print slam.get_pose_covariance()
