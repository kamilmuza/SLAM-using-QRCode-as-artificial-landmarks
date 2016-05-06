__author__ = 'Geist'
"""
    Tools to draw the robot position and landmarks...
"""
# TODO: Drawing capabilities...
from math import pi
import os

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from scipy.stats import chi2

plot_directory = os.path.dirname(os.path.abspath(__file__)) + "/plot"


def chi2inv_table():
    """
    #CHI2INVTABLE Lookup table of the inverse of the chi-square cdf.
    #   X = CHI2INVTABLE(P,V) returns the inverse of the chi-square cumu-
    #   lative distribution function (cdf) with V degrees of freedom at
    #   the value P. The chi-square cdf with V degrees of freedom, is
    #   the gamma cdf with parameters V/2 and 2.
    #
    #   Opposed to CHI2INV of the Matlab statistics toolbox which might
    #   be not part of your Matlab installation, this is a lookup table
    #   which has the side effect of being much faster than CHI2INV.
    #   However, as any lookup table is a collection of sample points,
    #   accuracy is smaller and between the sample points of the cdf, a
    #   linear interpolation is made.
    #
    #   Currently, the function supports the degrees of freedom V between
    #   1 and 10 and the probability levels P between 0 and 0.9999 in steps
    #   of 0.0001 and the level of 0.99999.
    #
    #   See also CHI2INV.
    """
    # TODO: implement, and check if is faster than sp.stats.chi2.ppf(...) ?????
    # a = chi2.ppf(0.95, 10)
    pass


def draw_robot(x, y, theta, size=0.1, color='y', type=3):
    """
        Draws the current robot position

        TYPE = 0 draws only a cross with orientation theta
        TYPE = 1 is a differential drive robot without contour
        TYPE = 2 is a differential drive robot with round shape
        TYPE = 3 is a round shaped robot with a line at theta
        TYPE = 4 is a differential drive robot with rectangular shape
        TYPE = 5 is a rectangular shaped robot with a line at theta

        Port of octave functions of the ekf framework, Cyrill course...
        color is one of: b,g,r,c,m,k,y
    """
    # Todo:only implemented type 3 for now...
    if type == 3:
        circle = plt.Circle((x, y), size, fc=color)
        # dx = size * np.sin(np.radians(theta))
        dx = size * np.cos(theta)
        dy = size * np.sin(theta)
        line = plt.Line2D([x, x + dx], [y, y + dy])
        plt.gcf().gca().add_patch(circle)
        plt.gcf().gca().add_line(line)
    plt.axis('scaled')


def draw_landmarks(lx, ly):
    plt.plot(lx, ly, 'r*', ms=5)
    plt.axis('scaled')


def draw_ellipse(x, y, theta, a, b, color='b'):
    """
        Draws an ellipse
        x,y center of ellipse
        theta, rotation angle in radians,
        a,b half width and height...
    """
    degree_angle = np.rad2deg(theta)
    # ellipse = mpl.patches.Ellipse((1,1),2,1,dangle)
    ellipse = mpl.patches.Ellipse((x, y), a * 2, b * 2, degree_angle, color=color, fill=False, lw=3)
    plt.gca().add_patch(ellipse)


def draw_prob_ellipse(x, c, alpha=0.1, color='b'):
    """
    Draws  the elliptic iso-probability contour of a Gaussian distributed bivariate random vector X
    at the significance level ALPHA. The ellipse is centered at X =[x; y]
    where C is the associated 2x2 covariance matrix.
    COLOR is a [r g b]-vector or a color string such as 'r' or 'g'.
    """
    # TODO: Check how to do this better, for now tune the alpha parameter...
    # original was 0.6
    sxx = c[0, 0]
    syy = c[1, 1]
    sxy = c[0, 1]
    a = np.sqrt(0.5 * (sxx + syy + np.sqrt((sxx - syy) ** 2 + 4 * sxy ** 2 + 0j)))  # always greater
    b = np.sqrt(0.5 * (sxx + syy - np.sqrt((sxx - syy) ** 2 + 4 * sxy ** 2 + 0j)))  # always smaller

    # Remove imaginary parts in case of neg. definite C
    # if not np.isreal(a):
    a = np.real(a)
    # if not np.isreal(b):
    b = np.real(b)
    a.shape = 1
    b.shape = 1
    a = a[0]
    b = b[0]
    # Scaling in order to reflect specified probability
    a = a * np.sqrt(chi2.ppf(alpha, 2))
    b = b * np.sqrt(chi2.ppf(alpha, 2))
    # Look where the greater half axis belongs to
    if sxx < syy:
        swap = a
        a = b
        b = swap
    # Calculate inclination (numerically stable)
    if sxx != syy:
        angle = 0.5 * np.arctan2(2 * sxy, (sxx - syy))
    elif sxy == 0:
        angle = 0  # angle doesn't matter
    elif sxy > 0:
        angle = pi / 4
    elif sxy < 0:
        angle = -pi / 4
    # Draw ellipse
    draw_ellipse(x[0], x[1], angle, a, b, color)
    # plt.axis('scaled')

def draw_state_for_me(timestep,x,y, z,path='/exp2/'):
    """
        Visualizes the state of the EKF SLAM algorithm.

        The resulting plot displays the following information:
        - map ground truth (black +'s)
        - current robot pose estimate (red)
        - current landmark pose estimates (blue)
        - visualization of the observations made at this time step (line between robot and landmark)
    """
    f = plt.gcf()
    f.clear()
    # f.hold()
    # draw the position of the landmarks

    #draw_landmarks(landmarks[0], landmarks[1])
    # draw the estimated position of the robot
   # pose = slam.get_pose()
    draw_robot(x, y, z)
    #draw the 2d gaussian of the estimated position of the robot
    #draw_prob_ellipse(pose, slam.get_pose_covariance(), color='r')
    #draw the estimated position of the landmarks and their gaussian
    # for (i, landmark) in enumerate(observedLandmarks):
    # print observedLandmarks
    
    # TODO: draw lines from robot to currently observed landmarks

    # Save Figure
    s = plot_directory + path + "robot_%03d.png" % timestep
    plt.axis('equal')
    lim = (-1,3)
    plt.ylim(lim)
    plt.xlim(lim)
    plt.savefig(s)



def draw_state(slam, landmarks, observedLandmarks, timestep, z):
    """
        Visualizes the state of the EKF SLAM algorithm.

        The resulting plot displays the following information:
        - map ground truth (black +'s)
        - current robot pose estimate (red)
        - current landmark pose estimates (blue)
        - visualization of the observations made at this time step (line between robot and landmark)
    """
    f = plt.gcf()
    f.clear()
    # f.hold()
    # draw the position of the landmarks

    draw_landmarks(landmarks[0], landmarks[1])
    # draw the estimated position of the robot
    pose = slam.get_pose()
    draw_robot(pose[0], pose[1], pose[2])
    #draw the 2d gaussian of the estimated position of the robot
    draw_prob_ellipse(pose, slam.get_pose_covariance(), color='r')
    #draw the estimated position of the landmarks and their gaussian
    # for (i, landmark) in enumerate(observedLandmarks):
    # print observedLandmarks
    for i in observedLandmarks:
        (l_x, l_y) = slam.get_landmark(i)
        plt.plot(l_x, l_y, 'b*', ms=5)
        # print slam.get_landmark_covariance(i)
        draw_prob_ellipse([l_x, l_y], slam.get_landmark_covariance(i))
    # TODO: draw lines from robot to currently observed landmarks

    # Save Figure
    s = plot_directory + "/ekf_%03d.png" % timestep
    plt.axis('equal')
    lim = (-1,3)
    plt.ylim(lim)
    plt.xlim(lim)
    plt.savefig(s)


def test_circle():
    circle1 = plt.Circle((0, 0), .2, color='r')
    circle2 = plt.Circle((.5, .5), .2, color='b')
    circle3 = plt.Circle((1, 1), .2, color='g', clip_on=False)
    fig = plt.gcf()
    fig.gca().add_artist(circle1)
    fig.gca().add_artist(circle2)
    fig.gca().add_artist(circle3)
    fig.savefig('plotcircles.png')

    # continuation example...
    circle1 = plt.Circle((0, 0), 2, color='r')
    # now make a circle with no fill, which is good for hilighting key results
    circle2 = plt.Circle((5, 5), .5, color='b', fill=False)
    circle3 = plt.Circle((10, 10), 2, color='g', clip_on=False)
    ax = plt.gca()
    ax.cla()  # clear things for fresh plot
    # change default range so that new circles will work
    ax.set_xlim((0, 10))
    ax.set_ylim((0, 10))
    # some data
    ax.plot(range(11), 'o', color='black')
    # key data point that we are encircling
    ax.plot((5), (5), 'o', color='y')

    fig.gca().add_artist(circle1)
    fig.gca().add_artist(circle2)
    fig.gca().add_artist(circle3)
    fig.savefig('plotcircles2.png')


def test_drawing():
    plt.axes()
    circle = plt.Circle((0, 0), 0.75, fc='y')
    plt.gca().add_patch(circle)

    line = plt.Line2D((2, 8), (6, 6), lw=2.5)
    plt.gca().add_line(line)

    dotted_line = plt.Line2D((2, 8), (4, 4), lw=5.,
                             ls='-.', marker='.',
                             markersize=50,
                             markerfacecolor='r',
                             markeredgecolor='r',
                             alpha=0.5)
    plt.gca().add_line(dotted_line)

    rectangle = plt.Rectangle((10, 10), 10, 10, fc='r')
    plt.gca().add_patch(rectangle)

    points = [[2, 1], [8, 1], [8, 4]]
    polygon = plt.Polygon(points)
    plt.gca().add_patch(polygon)

    points = [[2, 4], [2, 8], [4, 6], [6, 8]]
    line = plt.Polygon(points, closed=None, fill=None, edgecolor='r')
    plt.gca().add_patch(line)
    plt.axis('scaled')
    plt.show()




