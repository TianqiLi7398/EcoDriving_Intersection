from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
from sto_optimizer_update import stochastic_light, this_clock, next_clock, cost_time_step
from dum_controller import dummy_control
from smart_controller import smart_control
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import time
import pandas as pd
import copy
import os

def newline(p1, p2, color):
    # draw a line from p1=[x,y] to p2
    ax = plt.gca()
    ymin, ymax = p1[1], p2[1]
    xmin, xmax = p1[0], p2[0]

    l = mlines.Line2D([xmin, xmax], [ymin, ymax], color=color)
    ax.add_line(l)
    return l

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
        return alpha0 * 1000

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
    return Fuel_Consumption * 1000

alpha = 20    # the stop-and-go cost
idling_cost = alpha
x_init = 0
m = 5   # m = light.loc - x_init / dx
n = 23   # n = v_max - 1
t0 = 36                                    # initial clock, irrelavent to mdp generation
light_location = 50
timeset = [26, 5, 25]                      # a, red, yellow, green
timeset = [15, 3, 20]                      # b, red, yellow, green
timeset = [25, 5, 30]                      # c, red, yellow, green
dv = 1
##############################################
v_max = (n-1)*dv     
delta_x = (light_location - x_init) / m
a_min, a_max = -5, 8
w1 = 0.2   # work to do against friction
w2 = 0.1   # idling cost
w3 = 0.6   # accelerating cost weight

t0_set = np.linspace(0, sum(timeset), sum(timeset)//2 + 1)
destination_loc = (m + 1) * delta_x + x_init
vel_collection = [5, 10, 15, 20]
dt = 0.01
vel_collection = [20]
t0_set = [33]


for init_vel in vel_collection:

    result = pd.DataFrame(columns=["v0", "t0", "det", "sto", "dum"])

    for t0_ in t0_set:
        light = traffic_light(t0_, timeset, light_location)
        init_light = light.give_clock(0)
        time_real = np.linspace(dt, light.T, 100*light.T)

        final_time = light.T

        init_state = {
            "x": 0,
            "v": init_vel,
            "init_light": init_light,
            "t": 0,
            "must_happen": -1,
            "out_edge": [],
            "h": 99999}
        x0 = [0, init_vel, init_light, init_light]
        car = opt_vehicle(delta_x, v_max, a_max, a_min, light, x0, dt)
        
        # determined case
        x_det = copy.deepcopy(x0)
        cost_det = 0
        det_opt = dfs_optimizer(light, car)
        plan_det, cost, vel = det_opt.solver(x_det[0], x_det[1], dv)
        print(plan_det, cost, vel)
        # print(plan_det, vel)
        if np.isclose(vel[-2], 0):
            vel[-3] -= 0.2
        # print(plan_det, cost, vel)    
        action_det = []
        traj_det = []
        time_prof_det = []
        cost_det = 0
        idling_det = False
        # u_det = plan_det.pop(0)
        # action_det.append(u_det)
        vel.pop(0)
        for t in time_real:
            if x_det[0] >= destination_loc:
                break
            # if it meets the next grid, change to next plan
            if x_det[0] // det_opt.car.delta_x + 1 > len(action_det):
                x = find_nearest(x_det[0], det_opt.car.delta_x)
                
                a_plan = plan_det.pop(0)
                v_aim = vel.pop(0)
                # a_plan = det_opt.adjust_acc(v_aim, x_det[1])
                action_det.append(a_plan)
                # print(x_det, a_plan, v_aim, t)
                det_opt.dynamics_change(x_det, t)
            u_det = action_det[-1]  
            # go to dynamics
            if det_opt.light.location - x_det[0] < 0.5 and det_opt.light.location > x_det[0] and x_det[1] < 1:
                # close to intersection
                if x_det[2] == 3 and x_det[-1] == 3 and np.isclose(0, v_aim):
                    # red light waiting period
                
                    idling_det = True
                    x_det[1] = 0
                    u_det = 'stop'
                    det_opt.dynamics_change(x_det, t)
                elif x_det[2] == 1:
                    # change to green light, use next acc
                    u_det = plan_det[0]
                    det_opt.dynamics_change(x_det, t)
            
            cost_det += cost_ins(u_det, x_det[1], dt) * dt  
            x_det = det_opt.dynamics(x_det, u_det, t)
            traj_det.append(x_det[0])
            time_prof_det.append(t)
        

        

        plt.rcParams.update({'font.size': 15})
        plt.figure(figsize=(10, 8))

        # plot the trajectory
        # plt.plot(time_real, S[:, 0])


        # plt.plot(time_prof_sto, traj_sto, 'b')
        plt.plot(time_prof_det, traj_det, 'k')
        # plt.plot(time_prof_dum, traj_dum, 'g')
        
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
        plt.ylim([0, light_location * 1.5])
        #plt.legend(['optimal trajectory', 'red', 'green', 'yellow'])

        plt.title('Comparison between planning on single vehicle deterministic and stochastic traffic light, t0 = '+str(t0_))
        plt.grid(color='b', linestyle='-', linewidth=.2)
        prefix = str(x0[1])
        pic_name = 't0=' + str(t0_) + '_det.png'
        foldername = 'v0='+ str(x0[1])
        plt.show()
        # plt.savefig(os.path.join(os.getcwd(), 'pics', foldername, pic_name))
        print(t0_, 'saved!')

        print(cost_det)
        


    


