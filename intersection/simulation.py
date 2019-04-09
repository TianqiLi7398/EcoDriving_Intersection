# this is created in 2019/04/07 for a simple simulation of traffic intersection crossing
# Author: Tianqi Li

import numpy as np
import math

import matplotlib.lines as mlines
import matplotlib.pyplot as plt

def dynamics(s, u, dt):
#     state of the system
#     s_k = (y, v, l, l_p)
#     s_{k+1} = f(s_k, u_k)
    y = s[0] + u * dt **2 / 2 + s[1]*dt
    v = s[1] + u * dt
    lp = s[2]
    l = lp
    return [y, v, l, lp]



class traffic_light:
    def __init__(self, t0, timeset):
        self.t0 = t0
        self.red_dur = timeset[0]
        self.yel_dur = timeset[1]
        self.gre_dur = timeset[2]
        self.T = self.red_dur + self.yel_dur + self.gre_dur

    def give_clock(self, t):
        t =  ( t + self.t0) % self.T
        if t < self.gre_dur:
#             green light
            return 1
        elif t < self.gre_dur + self.yel_dur:
#             yellow light
            return 2
        else:
            # red light
            return 3

    def trafficline(self):
        t = self.T - (- self.t0 % self.T)
        if t < self.gre_dur:
#             green light
            green = [[0, 0], [self.gre_dur - t,0]]
            yellow = [[self.gre_dur - t,0], [self.gre_dur - t + self.yel_dur,0]]
            red = [[self.gre_dur - t + self.yel_dur,0], [self.T - t,0]]
        elif t < self.gre_dur + self.yel_dur:
#             yellow light
            yellow = [[0,0], [self.yel_dur - t, 0]]
            red = [[self.yel_dur - t,0], [self.yel_dur + self.red_dur - t,0]]
            green = [[self.yel_dur + self.red - t,0], [self.T - t,0]]

        else:
            # red light
            red = [[0,0], [self.red_dur - t,0]]
            green = [[self.red_dur - t,0], [self.red_dur - self.gre_dur - t,0]]
            yellow = [[self.red_dur - self.gre_dur - t,0], [self.T - t, 0]]
        return red, green, yellow


def newline(p1, p2, color):
    ax = plt.gca()
#     xmin, xmax = ax.get_xbound()

#     if(p2[0] == p1[0]):
#         xmin = xmax = p1[0]
#         ymin, ymax = ax.get_ybound()
#     else:
#         ymax = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmax-p1[0])
#         ymin = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmin-p1[0])
    ymin, ymax = p1[1], p2[1]
    xmin, xmax = p1[0], p2[0]

    l = mlines.Line2D([xmin,xmax], [ymin,ymax], color = color)
    ax.add_line(l)
    return l

def controller(s):
    v_opt = 13.6111 # for civic, 49 kph
    k = 0.1
    if s[0] < 0:
        # in front of the intersection
        if s[2] != 1:
             # not green light, calculate the acceleration to stop at intersection
            a =  s[1] ** 2 / (2 * s[0])
            if abs(s[0]) < 1:
                a = - s[1]/dt + 0.01
            return a
        else:
            return min(1, (v_opt - s[1]) / dt)
    else:
        return 0

v0 = 10 * 0.44704 # mpg to m/s
L = 200 # sensor range 200 m
t0 = 25
v_opt = 13.6111 # for civic, 49 kph
timeset = [40, 4, 35]
light = traffic_light(t0, timeset)
s0 = [-L, v0, light.give_clock(0), None]

timeline = np.linspace(0, 100, 1000)

S = []
U = []
s = s0
dt = 0.1
time_green, time_yellow, time_red =[],[],[]
for t in timeline:
    u = controller(s)
    s_ = dynamics(s, u, dt)
    s_[2] = light.give_clock(t)
    S.append(s_)
    U.append(u)
    s = s_
    if s[2] == 1:
        time_green.append(t)
    elif s[2] == 2:
        time_yellow.append(t)
    else:
        time_red.append(t)
S = np.asarray(S)

# plot the result
plt.rcParams.update({'font.size': 15})
plt.plot(timeline, S[:,0])
red, green, yellow = light.trafficline()
newline(red[0],red[1],'r')
newline(green[0],green[1],'g')
newline(yellow[0],yellow[1],'y')
plt.xlabel('time/sec')
plt.ylabel('position/m')
plt.legend(['trajectory', 'red', 'green', 'yellow'])
plt.title('Simple simulation of vehilce going through intersection')
plt.grid(color='b', linestyle='-', linewidth=.2)
plt.show()
