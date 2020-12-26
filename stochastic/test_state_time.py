from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
from sto_optimizer_update import stochastic_light, this_clock, next_clock
import numpy as np
from state_time_opt import state_space
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import time

import copy
import os



idling_cost = 0
x_init = 0
m = 5   # m = light.loc - x_init / dx
n = 23   # n = v_max - 1
t0 = 36                                    # initial clock, irrelavent to mdp generation
light_location = 50
timeset = [26, 5, 25]                      # a, red, yellow, green
# timeset = [15, 3, 20]                      # b, red, yellow, green
# timeset = [25, 5, 30]                      # c, red, yellow, green
dv, dt = 1, 0.5
##############################################
v_max = (n-1)*dv     
delta_x = (light_location - x_init) / m
a_min, a_max = -5, 8
t0_ = 28.0

light = traffic_light(t0_, timeset, light_location)
init_light = light.give_clock(0)
init_vel = 15.0

final_time = light.T

init_state = {
    "x": x_init,
    "v": init_vel,
    "init_light": init_light,
    "t": 0,
    "must_happen": -1,
    "out_edge": [],
    "h": 99999}
x0 = [0, init_vel, init_light, init_light]
car = opt_vehicle(delta_x, v_max, a_max, a_min, light, x0, dt)

obj = state_space(light, car)
obj.solver(0.0, init_vel)