from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
from sto_optimizer_update import stochastic_light, this_clock, next_clock
from state_time_opt import state_space
from smart_controller import smart_control
import numpy as np
import json
import time
import pandas as pd
import copy
import os

def find_nearest(x, n):
    # given x, find nearst integer times of n
    m = x // n

    if x % n > n/2:
        return n*(m + 1)
    else:
        return n*m
    
def cost_ins(a, v, dt):
    # Speed v: m/s
    # acceleration a: m/s^2
    # output: originally is L/s, change it to cm^3/s for computation convinence 
    alpha0 = 0.0006289
    alpha1 = 2.676e-05
    alpha2 = 1e-06
    if a == 'stop':
        return alpha0 * 1000 * dt

    vel=float(v)
    vel = vel * 3.6 # convert m/s -> km/h
    acc=float(a)
    Chig = 1
    Cr = 1.75 
    c1 = 0.0328
    C2 = 4.575
    Afront = 2.28
    etaD = 0.92
    m = 1470; # Kg
    Cdrag = 0.28
    widle = 700; # (rpm)
    d = 2.5
    N = 4
    FEhigh = 35
    FEcity=25
    #Fhwy=float(38.6013)*float((1.3466/float(FEhigh))+0.001376)
    #G=0 #grade
    #Calculate the total Resistance forces.
    R = float((1.2256 / 25.92) *float(Cdrag) * float(Chig) *float(Afront) * (vel*vel) +
    (9.8066 *float(m) * float(Cr) *( (float(c1) * vel +float(C2)) / 1000)))
    #Input the calibrated parameters values.
    P =float(((R + 1.04 * m * acc) / (3600 * etaD)) * vel)
    #alpha0 = float((Pmf0 * widle * d * 3.42498) / (22164 * Q * N))

    Fuel_Consumption =(alpha0 + alpha1 * P + alpha2 * (P *P))
    if P < 0:
        Fuel_Consumption = float(alpha0)
    #print(alpha0,alpha1,alpha2)
    return Fuel_Consumption * 1000 * dt


idling_cost = 0
x_init = 0
m = 5   # m = light.loc - x_init / dx
n = 23   # n = v_max - 1
light_location = 50.0
timeset = [26, 5, 25]                      # a, red, yellow, green
# timeset = [15, 3, 20]                      # b, red, yellow, green
# timeset = [25, 5, 30]                      # c, red, yellow, green
trafficFolderName = str(timeset[0]) + '_' + str(timeset[1]) + '_' + str(timeset[2])
dv = 1
##############################################
v_max = (n-1)*dv     
delta_x = (light_location - x_init) / m
a_min, a_max = -5, 8

t0_set = np.linspace(0, sum(timeset), sum(timeset)//2 + 1)
destination_loc = (m + 1) * delta_x + x_init
vel_collection = [5, 10, 15, 20]
# vel_collection = [15]
dt = 0.01
v_star = 7
# vel_collection = [5]
# t0_set = [24

det_time_list, opt_time_list = [], []


for init_vel in vel_collection:
    result = {"det":[], "opt": []}
    for t0_ in t0_set:
    # for lll in range(1):
    #     t0_ = t0_set[[0, 2][lll]]
        print("t0 = ", t0_)
        light = traffic_light(t0_, timeset, light_location)
        init_light = light.give_clock(0)
        time_real = np.linspace(dt, light.T, 100*light.T)

        final_time = 1.5 * light.T

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
        # B. determined case
        x_det = copy.deepcopy(x0)
        cost_det = 0
        det_opt = dfs_optimizer(light, car)
        det_start = time.time()
        plan_det, cost, vel = det_opt.solver(x_det[0], x_det[1], dv)
        det_delta_t = time.time() - det_start
        

        # D. state_time control
        obj = state_space(light, car)
        opt_start = time.time()
        st_plan = obj.solver(0.0, init_vel)
        opt_delta_t = time.time() - opt_start

        det_time_list.append(det_delta_t)
        opt_time_list.append(opt_delta_t)
        

        print(det_delta_t)
        print(opt_delta_t)
    
    result["det"] = det_time_list
    result["opt"] = opt_time_list
    
    filename = str(init_vel) + '-' + str(light.red_dur)+'-'+str(light.yel_dur)+'-'+str(light.gre_dur)+'_runtime.json'
    pathname = os.path.join(os.getcwd(), 'data', filename)
    with open(pathname, 'w') as outfiles:
        json.dump(result, outfiles, indent=4)
    # result.to_csv(os.path.join(os.getcwd(), 'data', filename), sep='\t')
    print('v0 = ', x0[1], ' json result saved!')


