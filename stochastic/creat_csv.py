from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
from sto_optimizer import stochastic_light, this_clock, next_clock, cost_time_step
from dum_controller import dummy_control
import numpy as np
import pandas as pd

import time

import copy




start_point = time.time()
x_init = 0
v_init = 15
m = 5  # m = light.loc - x_init / dx
n = 23 # n = v_max - 1


t0 = 36                                    # initial clock, irrelavent to mdp generation
light_location = 50
timeset = [26, 5, 25]                      # red, yellow, green
idling_cost = 10*8
true_idling_cost = 1                    # 0.1 unit/s
dv = 1
run_light_penalty = 999
##############################################



v_max = (n-1)*dv
start_point = time.time()
                     # red, yellow, green

delta_x = (light_location - x_init) / m
epslon = 10 ** -6

delta_t_min = delta_x / v_max
a_min, a_max = -5, 8
w1, w2 = 1/8, 1/8

t0_set = np.linspace(0, sum(timeset), sum(timeset)//2 + 1)



destination_loc = (m + 1) * delta_x + x_init

vel_collection = [5,10,15,20, 22]
# vel_collection = [15]


vel_collection = [20]
t0_ = 28
init_vel = 10


t0_ = round(t0_, 1)

light = traffic_light(t0_, timeset, light_location)
init_light = light.give_clock(0)
init_state = {
    "x": x_init,
    "v": init_vel,
    "init_light": init_light,
    "t": 0,
    "must_happen": -1,
    "out_edge": [],
    "h": 99999}

x0 = [0, init_vel, light.give_clock(0), light.give_clock(0)]
car = opt_vehicle(delta_x, v_max, a_max, a_min, light, x0)

#####################################################################
# generate solutions



sto_opt = stochastic_light(light, init_state, m, n, v_max, car, idling_cost)
sto_opt.load_prior()
