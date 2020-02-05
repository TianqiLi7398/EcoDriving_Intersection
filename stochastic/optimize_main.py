import numpy as np
from util import opt_vehicle, traffic_light
from optimizer import dfs_optimizer
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import copy
import math


def newline(p1, p2, color):
    # draw a line from p1=[x,y] to p2
    ax = plt.gca()
    ymin, ymax = p1[1], p2[1]
    xmin, xmax = p1[0], p2[0]

    l = mlines.Line2D([xmin, xmax], [ymin, ymax], color=color)
    ax.add_line(l)
    return l


delta_x = 10
epslon = 10 ** -6
v_max = 22
delta_t_min = delta_x / v_max
a_min, a_max = -5, 8
w1, w2 = 1/8, 1/8

timeset = [20, 5, 25]                       # red, yellow, green
t0 = 40
light_location = 50
light = traffic_light(t0, timeset, light_location)
x0 = [0, 15, light.give_clock(0), light.give_clock(0)] # [x0, v0, light, previous light] is redundant here, but just keep it. TODO: simplify this
dt = 0.01


car = opt_vehicle(delta_x, v_max, a_max, a_min, light, x0)
optimizer = dfs_optimizer(light, car)
cost, U, vel = optimizer.solver(1)
print(cost, U, vel)

# plot the result trajectory
time_real = np.linspace(dt, light.T, 100*light.T)
final_time = light.T
# create the trajcetory
traj = []
vel = []
x = x0
for t in time_real:
    #     traj.append(x)

    u = optimizer.controller(x)
    x = optimizer.dynamics(x, u)
    x[2] = light.give_clock(t)
    traj.append(x[0])
    vel.append(x[1])
    # print(t, x, u)

plt.rcParams.update({'font.size': 15})
plt.figure(figsize=(10, 8))
# plot the trajectory
# plt.plot(time_real, S[:, 0])
plt.plot(time_real, traj, 'g')
# plt.plot(time_real, vel, 'b')  

red, green, yellow = light.trafficline(final_time)
red_num, green_num, yel_num = len(red), len(green), len(yellow)
for j in range(int(red_num/2)):

    newline(red[2*j], red[2*j + 1], 'r')
for j in range(int(green_num/2)):
    newline(green[2*j], green[2*j + 1], 'g')
for j in range(int(yel_num/2)):
    newline(yellow[2*j], yellow[2*j + 1], 'y')

plt.xlabel('time/sec')
plt.ylabel('position/m')
plt.xlim([0, light.T])
plt.ylim([0, light_location * 1.1])
plt.legend(['optimal trajectory', 'red', 'green', 'yellow'])

plt.title('Simple simulation of vehilce going through intersection')
plt.grid(color='b', linestyle='-', linewidth=.2)
# plt.savefig('through_intersec.png')
plt.show()
