__author__ = 'Geist'
from prob import *
# Motion models


# State vector X = [x_coordinate, y_coordinate, orientation ]


#Velocity model
# a1 = a2 = a3 = a4 = a5 = a6 = 0.5
# var1 = a1 * abs(Ut[0]) + a2 * abs(Ut[1])
# var2 = a3 * abs(Ut[0]) + a4 * abs(Ut[1])
# var3 = a5 * abs(Ut[0]) + a6 * abs(Ut[1])
# Action vector Ut = [vt,wt]
def velocity_model(Xt, Ut, Xt0, delta_t):
    #parameters a1...a6 model intrinsic, need to calculate...
    a1 = a2 = a3 = a4 = a5 = a6 = 0.5
    var1 = a1 * abs(Ut[0]) + a2 * abs(Ut[1])
    var2 = a3 * abs(Ut[0]) + a4 * abs(Ut[1])
    var3 = a5 * abs(Ut[0]) + a6 * abs(Ut[1])

    u = 1 / 2 * ((Xt0[0] - Xt[0]) * cos(Xt0[2]) + (Xt0[1] - Xt[1]) * sin(Xt0[2]))
    u /= ((Xt0[1] - Xt[1]) * cos(Xt0[2]) + (Xt0[0] - Xt[0]) * sin(Xt0[2]))

    xc = (Xt0[0] + Xt[0]) / 2 + u * (Xt0[1] - Xt[1])
    yc = (Xt0[1] + Xt[1]) / 2 + u * (Xt0[0] - Xt[0])
    r = sqrt((Xt0[0] - Xt[0]) ** 2 + (Xt0[1] - Xt[1]) ** 2)
    delta_alfa = atan2(Xt[1] - yc, Xt[0] - xc) - atan2(Xt0[1] - yc, Xt0[0] - xc)
    new_w = delta_alfa / delta_t
    new_v = new_w * r
    g = (Xt[2] - Xt0[2]) / delta_t - new_w
    return normal_dist(Ut[0] - new_v, var1) * normal_dist(Ut[1] - new_w, var2) * normal_dist(g, var3)


    #Odometry model


def sample_velocity_model(Ut, Xt0, delta_t):
    a1 = a2 = a3 = a4 = a5 = a6 = 0.5
    var1 = a1 * abs(Ut[0]) + a2 * abs(Ut[1])
    var2 = a3 * abs(Ut[0]) + a4 * abs(Ut[1])
    var3 = a5 * abs(Ut[0]) + a6 * abs(Ut[1])

    v = Ut[0] + sample_normal_dist(var1)
    w = Ut[1] + sample_normal_dist(var2)
    g = sample_normal_dist(var3)
    new_x = Xt0[0] - v / w * sin(Xt0[2]) + v / w * sin(Xt0[2] + w * delta_t)
    new_y = Xt0[1] + v / w * cos(Xt0[2]) - v / w * cos(Xt0[2] + w * delta_t)
    new_alfa = Xt0[2] + w * delta_t + g * delta_t
    return [new_x, new_y, new_alfa]


#Odometry model
# Technically, odometry are sensor measurements, not controls.
# To keep the state space small, it is therefore common to simply consider the odometry
# as if it was a control signal.
# At time t, the correct pose of the robot is modeled by the random variable xt.
# The robot odometry estimates this pose; however, due to drift and slippage
# there is no fixed coordinate transformation between the coordinates used by
# the robots internal odometry and the physical world coordinates.
# Accepts as an input an initial pose xt-1, a pair of poses ut = (xt-1, xt)
# obtained from the robots odometry, and a hypothesized final pose xt.
# It outputs the numerical probability p(xt |ut,xt-1).

# Ut = [Xt-1, Xt] as reported from odometry

def odometry_model(Xt, Ut, Xt0, delta_t):
    a1 = a2 = a3 = a4 = 0.5

    xb = Ut[0][0]
    yb = Ut[0][1]
    ob = Ut[0][2]
    xbp = Ut[1][0]
    ybp = Ut[1][1]
    obp = Ut[1][2]
    x = Xt0[0]
    y = Xt0[1]
    o = Xt0[2]
    xp = Xt[0]
    yp = Xt[1]
    op = Xt[2]

    rot1 = atan2(ybp - yb, xbp - xb) - ob
    tran = sqrt((xb - xbp) ** 2 + (yb - ybp) ** 2)
    rot2 = obp - ob - rot1

    rot1p = atan2(yp - y, xp - x) - o
    tranp = sqrt((x - xp) ** 2 + (y - yp) ** 2)
    rot2p = op - o - rot1p

    var1 = a1 * rot1p + a2 * tranp
    var2 = a3 * tranp + a4 * (rot1p + rot2p)
    var3 = a1 * rot2p + a2 * tranp

    p1 = normal_dist(rot1 - rot1p, var1)
    p2 = normal_dist(tran1 - tran1p, var2)
    p3 = normal_dist(rot2 - rot2p, var3)

    return p1 * p2 * p3


def sample_odometry_model(Ut, Xt0, delta_t):
    a1 = a2 = a3 = a4 = 0.5

    xb = Ut[0][0]
    yb = Ut[0][1]
    ob = Ut[0][2]
    xbp = Ut[1][0]
    ybp = Ut[1][1]
    obp = Ut[1][2]
    x = Xt0[0]
    y = Xt0[1]
    o = Xt0[2]

    rot1 = atan2(ybp - yb, xbp - xb) - ob
    tran = sqrt((xb - xbp) ** 2 + (yb - ybp) ** 2)
    rot2 = obp - ob - rot1

    var1 = a1 * rot1 + a2 * tran
    var2 = a3 * tran + a4 * (rot1 + rot2)
    var3 = a1 * rot2 + a2 * tran

    rot1 -= sample_normal_dist(var1)
    tran -= sample_normal_dist(var2)
    rot2 -= sample_normal_dist(var3)

    x += tran * cos(o + rot1)
    y += tran * sin(o + rot1)
    o += rot1 + rot2

    return [x, y, o]