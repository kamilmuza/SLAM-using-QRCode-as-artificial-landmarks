n__author__ = 'Geist'
# Base filters implementation...


# EKF localization, is a special case of Markov localization.
# EKF localization represent beliefs bel(xt) by their their
# first and second moment, that is, the mean ut and the
# covariance Et. Our EKF localization algorithm assumes that the
# map is represented by a collection of features.