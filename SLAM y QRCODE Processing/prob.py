__author__ = 'Geist'
from math import *
import random

# some probability functions


# Gaussian distribution with mean = 0 and variance = b
def normal_dist(a, b):
    return exp(-0.5 * (a ** 2) / b) / sqrt(2 * pi * b)


# Gaussian distribution with mean = mu and variance = var
def Gaussian(x, var, mu=0):
    return exp(-0.5 * ((mu - x) ** 2) / var) / sqrt(2.0 * pi * var)


# Triangular distribution with mean = 0 and variance = b
def triangular_dist(a, b):
    if abs(a) > sqrt(6 * b):
        return 0
    else:
        return (sqrt(6 * b) - abs(a)) / (6 * b)


#Algorithm for sampling from (approx) distributions
def sample_normal_dist(b):
    p = 0
    for i in xrange(12):
        p += random.uniform(-1, 1)
    return p * b / 6


def sample_normal_dist2(b):
    p = 0
    for i in xrange(12):
        p += random.uniform(-b, b)
    return p / 2


def sample_triangular_dist(b):
    return b * random.uniform(-1, 1) * random.uniform(-1, 1)


def sample_triangular_dist2(b):
    return sqrt(6) * (random.uniform(-b, b) + random.uniform(-b, b)) / 2


#Exponential distribution and normalizer
def exponential_dist(x, alfa):
    return alfa * exp(-alfa * x)


def normalized_exponential_dist(x, alfa, x_expected):
    if 0 <= x <= x_expected:
        return exponential_dist(x, alfa) / (1 - exp(-alfa * x_expected))
    else:
        return 0
