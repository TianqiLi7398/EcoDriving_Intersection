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

alpha = 20    # the stop-and-go cost
idling_cost = alpha
x_init = 0
m = 5   # m = light.loc - x_init / dx
n = 23   # n = v_max - 1
t0 = 36                                    # initial clock, irrelavent to mdp generation
light_location = 50
timeset = [26, 5, 25]                      # red, yellow, green
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
# vel_collection = [5]
# t0_set = [28]
opt_vel_list = np.linspace(1, 22, 22)


for init_vel in vel_collection:
    # print("v_0 = ", init_vel)
    # result = pd.DataFrame(columns=["v0", "t0", "det", "sto", "dum"])
    for v_star in opt_vel_list:    
        v_star_cost = 0
        for t0_ in t0_set:
            # print("t_0 = ", t0_)
            light = traffic_light(t0_, timeset, light_location)
            init_light = light.give_clock(0)
            time_real = np.linspace(dt, light.T, 100*light.T)

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

            # dummy control
            dum_con = smart_control(light, car, a_max, a_min, opt_vel=v_star)
            traj_dum = []
            time_prof_dum = []
            cost_dum = 0
            x_dum = copy.deepcopy(x0)
            idling_dum = False
            for t in time_real:
                
                if x_dum[0] >= destination_loc:
                    break

                u_dum = dum_con.controller(x_dum)

                cost_dum += cost_ins(u_dum, x_dum[1], dt)
                x_dum = dum_con.dynamics(x_dum, u_dum)

                # update the light signal

                x_dum[2] = light.give_clock(t)
                traj_dum.append(x_dum[0])
                

                time_prof_dum.append(t)
            
            v_star_cost += cost_dum
        print("c(v0= %s, v*= %s) = %s"%(init_vel, v_star, v_star_cost))

        # plt.rcParams.update({'font.size': 15})
        # plt.figure(figsize=(10, 8))

        # # plot the trajectory
        # # plt.plot(time_real, S[:, 0])


        # # plt.plot(time_prof_sto, traj_sto, 'b')
        # # plt.plot(time_prof_det, traj_det, 'k')
        # plt.plot(time_prof_dum, traj_dum, 'g')
        # plt.legend(['sto', 'det', "smart"])
        # # plt.plot(time_real, vel, 'b')

        # red, green, yellow = light.trafficline(final_time)
        # red_num, green_num, yel_num = len(red), len(green), len(yellow)
        # for j in range(int(red_num/2)):
        #     newline(red[2*j], red[2*j + 1], 'r')
        # for j in range(int(green_num/2)):
        #     newline(green[2*j], green[2*j + 1], 'g')
        # for j in range(int(yel_num/2)):
        #     newline(yellow[2*j], yellow[2*j + 1], 'y')

        # plt.xlabel('time/sec')
        # plt.ylabel('position/m')
        # plt.xlim([0, light.T])
        # plt.ylim([0, light_location * 1.5])
        # #plt.legend(['optimal trajectory', 'red', 'green', 'yellow'])

        # plt.title('Comparison between planning on single vehicle deterministic and stochastic traffic light, t0 = '+str(t0_))
        # plt.grid(color='b', linestyle='-', linewidth=.2)
        # prefix = str(x0[1])
        # pic_name = 't0=' + str(t0_) + '_smt.png'
        # foldername = 'v0='+ str(x0[1])
        # plt.savefig(os.path.join(os.getcwd(), 'pics', foldername, pic_name))
        # print(t0_, 'saved!')
    
        


    
    # filename = str(0) + '-' + str(x0[1]) + '-' + str(light.red_dur)+'-'+str(light.yel_dur)+'-'+str(light.gre_dur)+'_result.csv'
    # result.to_csv(os.path.join(os.getcwd(), 'data', filename), sep='\t')
    # print('v0 = ', x0[1], ' csv result saved!')