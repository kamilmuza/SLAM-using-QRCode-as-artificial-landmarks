__author__ = 'Geist'
from slam import *

# Test GraphSlam. One dimension problem, 3 movement, 1 landmark. Starts at position -3. Landmark at position 7.
init = -3
move1 = 5
move2 = 3
z1 = 10
z2 = 5
z3 = 2

noise = 1

slam = GraphSlam(1, 1, init, 3)

slam.update_landmark(1, z1, noise)
slam.update_pose(move1, noise)

slam.update_landmark(1, z2, noise)
slam.update_pose(move2, noise)

slam.update_landmark(1, z3, noise)

print slam.get_map()
print '\n'
slam = GraphSlamOnline(1, 1, init)

slam.update_landmark(1, z1, noise)
slam.update_pose(move1, noise)

slam.update_landmark(1, z2, noise)
slam.update_pose(move2, noise)

slam.update_landmark(1, z3, noise)

print slam.get_map()