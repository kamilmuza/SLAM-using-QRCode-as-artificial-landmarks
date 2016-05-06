from math import pi

import numpy as np


__author__ = 'Geist'
# TODO: Manage exception. Fail gracefully :)...


def fibonacci(x):
    if x < 0:
        raise Exception('Value error, x must be greater or equal to 0')
    if x <= 1:
        return 1
    return fibonacci(x - 1) + fibonacci(x - 2)


def read_data(path='data/sensor_data.dat'):
    # Based on the frameworks from Cyrill SLAM Course
    """
    Reads the odometry and sensor readings from a file.

    The data is returned in a structure where the u_t and z_t are stored
    within a single entry. A z_t can contain observations of multiple
    landmarks.

    Usage:
    - access the readings for timestep i:
    data.[i]
    this returns a structure containing the odometry reading and all
    landmark observations, which can be accessed as follows
    - odometry reading at timestep i:
    data.[i]['odometry']
    - sensor reading at timestep i:
    data.[i]['sensor']

    Odometry readings have the following fields:
    - r1 : rotation 1
    - t  : translation
    - r2 : rotation 2
    which correspond to the identically labeled variables in the motion
    mode.

    Sensor readings can again be indexed and each of the entries has the
    following fields:
    - id      : id of the observed landmark
    - range   : measured range to the landmark
    - bearing : measured angle to the landmark (you can ignore this)

    Examples:
    - Translational component of the odometry reading at timestep 10
    data.[10]['odometry']['t']
    - Measured range to the second landmark observed at timestep 4
    data.[4]['sensor'][2]['range']

    @type path: str
    @param path: path to the file to parse
    @return data: structure containing the parsed information
    """
    first = True
    data = []
    sensor = []
    with open(path) as my_file:
        for line in my_file:
            tokens = line.split()
            if 'ODOMETRY' in tokens[0]:
                if not first:
                    data.append({'sensor': sensor, 'odometry': odometry})
                    sensor = []
                if first:
                    first = False
                odometry = {'r1': float(tokens[1]), 't': float(tokens[2]), 'r2': float(tokens[3])}
            elif 'SENSOR' in tokens[0]:
                reading = {'id': int(tokens[1]), 'range': float(tokens[2]), 'bearing': float(tokens[3])}
                sensor.append(reading)
    # return data[1:]
    return data


def read_world(path='data/world.dat'):
    # Based on the frameworks from Cyrill SLAM Course
    """
    Reads the world definition and returns a structure of landmarks.

    Each landmark contains the following information:
    - id : id of the landmark
    - x  : x-coordinate
    - y  : y-coordinate

    Examples:
    - Obtain x-coordinate of the 5-th landmark
    landmarks[5]['x']

    @type path: str
    @param path: path to the file to parse
    @return landmarks: structure containing the parsed information
    """
    landmarks = []
    with open(path) as my_file:
        for line in my_file:
            tokens = line.split()
            landmark = {'id': int(tokens[0]), 'x': float(tokens[1]), 'y': float(tokens[2])}
            landmarks.append(landmark)
    return landmarks


def normalize_angle(phi):
    # TODO: TEST if it works for...
    """
    Normalize phi to be between -pi and pi
    @param phi: angle, list or vector(ndarray) of angles
    @return: normalized angles [-pi, pi]
    """
    if not isinstance(phi, np.ndarray):
        phi = np.array(phi)
    if not phi.shape:
        phi.shape = 1
    while (phi > pi).any():
        phi[phi > pi] -= 2 * pi
    while (phi < -pi).any():
        phi[phi < -pi] += 2 * pi
    if len(phi) == 1:
        return phi[0]
    return phi


def normalize_bearings(z):
    if not isinstance(z, np.ndarray):
        z = np.array(z)
    z[1::2] = normalize_angle(z[1::2])
    return z


if __name__ == "__main__":
    d = read_data('data/sensor.dat')
    w = read_world('data/world.dat')

    print "data:\n"
    for (i, step) in enumerate(d):
        print "TIMESTEP:" + str(i)
        print "Odometry"
        print "r1:%02.8f\t t:%02.8f\t r2:%02.8f\t" % (
        step['odometry']['r1'], step['odometry']['t'], step['odometry']['r2'])
        print "Sensor"
        for (j, o) in enumerate(step['sensor']):
            print "Observation:%d id:%d\t range:%02.8f\t bearing:%02.8f\t" % (j, o['id'], o['range'], o['bearing'])

    print "world:\n"
    for landmark in w:
        print "id:%d\t,x:%d\t,y:%d\t" % (landmark['id'], landmark['x'], landmark['y'])
        # print "id:%d\t,x:%02.8f\t,y:%02.8f\t" % (landmark['id'], landmark['x'], landmark['y'])