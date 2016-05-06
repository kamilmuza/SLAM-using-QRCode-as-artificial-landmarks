import sys

import numpy as np


__author__ = 'Geist'

_filename = sys.argv[0]
_usage = """
Usage: %s A=10 p=5 n=730
Usage of the script """ % _filename


class OGMap:
    def __init__(self, xm, ym, r):
        self.xm = xm if xm != 0 else 100.0
        self.ym = ym if ym != 0 else self.xm
        self.r = r if r != 0 else 0.15
        self.xn = int(self.xm / self.r) + 1
        self.yn = int(self.ym / self.r) + 1
        self.grid = np.mat(np.zeros((self.xn, self.yn)) + 0.5)

    def cell(self, x, y):
        # Returns the probability of occupancy of the cell in X,Y
        x = int(x / self.r)
        y = int(y / self.r)
        return self.grid[x, y]

    def update_cell(self, x, y, value):
        # Updates the cell in X,Y according to the newest measurement.
        # value = p(c|X(t),Z(t)) and depends on the sensor
        # pc is the recursive term => p(c| X(1:t-1),Z(1:t-1))
        x = int(x / self.r)
        y = int(y / self.r)
        pc = self.grid[x, y]  # current probability
        ud = (1.0 - value) / value  # Update formula, see Stachniss 2006 (phd)
        ud *= (1.0 - pc) / pc
        ud = 1.0 / (ud + 1)
        self.grid[x, y] = ud


class CoverageMap:
    def __init__(self, xm, ym, r):
        self.xm = xm if xm != 0 else 100.0
        self.ym = ym if ym != 0 else self.xm
        self.r = r if r != 0 else 0.15
        self.xn = int(self.xm / self.r) + 1
        self.yn = int(self.ym / self.r) + 1
        self.grid = np.mat(np.zeros((self.xn, self.yn)) + 0.5)

    def cell(self, x, y):
        # Returns the probability of occupancy of the cell in X,Y
        x = int(x / self.r)
        y = int(y / self.r)
        return self.grid[x, y]

    def update_cell(self, x, y, value):
        pass


class ReflectionMap:
    def __init__(self, xm, ym, r):
        self.xm = xm if xm != 0 else 100.0
        self.ym = ym if ym != 0 else self.xm
        self.r = r if r != 0 else 0.15
        self.xn = int(self.xm / self.r) + 1
        self.yn = int(self.ym / self.r) + 1
        self.grid = np.mat(np.zeros((self.xn, self.yn)) + 0.5)

    def cell(self, x, y):
        # Returns the probability of occupancy of the cell in X,Y
        x = int(x / self.r)
        y = int(y / self.r)
        return self.grid[x, y]

    def update_cell(self, x, y, value):
        pass


def _verify():
    pass


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print _usage
    elif len(sys.argv) == 2 and sys.argv[1] == 'verify':
        _verify()
    else:
        pass