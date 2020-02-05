# This is created on Jan 14, 2020 by Tianqi Li
# This helped create MDP for stochastic traffic intersection

# 1. Build the skeleton, which is a tree with root of initial state, branching
# to all its solutions;
# 2. Build MDP, with each edge reaching from s to another state s', it calculate utility
#     h(s) = min_s_i p(g, t+dt| g, t)h(s_i) + p(y, t+dt| g, t)cost(s_i, y)
# also the policy
#     pi(s) = argmin_i p(g, t+dt| g, t)h(s_i) + p(y, t+dt| g, t)cost(s_i, y)
# where the cost(s_i, y) is calculated by the full information intersection optimizer 
from util import opt_vehicle, traffic_light, vertex
import numpy as np


delta_x = 10
epslon = 10 ** -6
v_max = 22
delta_t_min = delta_x / v_max
a_min, a_max = -5, 8
w1, w2 = 1/8, 1/8

timeset = [20, 5, 25]                       # red, yellow, green
t0 = 40                                     # initial clock
light_location = 50
light = traffic_light(t0, timeset, light_location)
x0 = [0, 15, light.give_clock(0), light.give_clock(0)]
dt = 0.01

# 1. build the graph
init_ver = vertex(0, 15, 0)     # initial state

# start from the init_ver, expand all possible vertices
# define m+1 as the number of x lattice, n as number of v lattice
m = 5
n = 23
dx = (light_location - init_ver.x) / m
loc_grid = np.linspace(dx, light_location + dx, m + 1)
vel_grid = np.linspace(0, v_max, n)

print(vel_grid)
