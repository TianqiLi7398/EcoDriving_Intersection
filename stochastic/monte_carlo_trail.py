from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
from sto_optimizer_update import stochastic_light, this_clock, next_clock
from dum_controller import dummy_control
from smart_controller import smart_control
import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
# import matplotlib.lines as mlines
import time
import pandas as pd
import copy
import os
import random

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

    vel= float(v)
    vel = vel * 3.6 # convert m/s -> km/h
    acc= float(a)
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

init_time = time.time()

idling_cost = 0
x_init = 0
m = 5   # m = light.loc - x_init / dx
n = 23   # n = v_max - 1
t0 = 36                                    # initial clock, irrelavent to mdp generation
light_location = 50
# timeset = [26, 5, 25]                      # a, red, yellow, green
timeset = [15, 3, 20]                      # b, red, yellow, green
# timeset = [25, 5, 30]                      # c, red, yellow, green
trafficFolderName = str(timeset[0]) + '_' + str(timeset[1]) + '_' + str(timeset[2])
dv = 1
##############################################
v_max = (n-1)*dv     
delta_x = (light_location - x_init) / m
a_min, a_max = -5, 8
destination_loc = (m + 1) * delta_x + x_init
vel_collection = [5, 10, 15, 20]
dt = 0.01
# vel_collection = [5, 10, 15, 20]
# t0_set = [24]
T = sum(timeset)
trail_num = 500

for init_vel in vel_collection:

    result = pd.DataFrame(columns=["v0", "t0", "det", "sto", "dum"])

    for trail in range(trail_num):
        t0_ = random.uniform(0, T)
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
        
        
        # A. Stochastic part 
        
        sto_opt = stochastic_light(light, init_state, m, n, v_max, car, idling_cost)
        sto_opt.load_prior()
        time_prof_sto = []
        traj_sto = []
        x_sto = copy.deepcopy(x0)
        # u_sto = []
        action_mdp = [sto_opt.mdp[(sto_opt.mdp["x"]==x0[0]) & (sto_opt.mdp["v"]==x0[1]) & (sto_opt.mdp["init_light"]==init_light) & (sto_opt.mdp["t"]==0)]["best_action"].values[0]]
        light_changed = False
        fixed_policy = []
        cost_sto = 0
        compansate_light = 0
        fixed_policy_bool = False
        idling_sto = False

        
        for t in time_real:
            # if x_sto[1] > 0:
            #     print(x_sto, t)
            # 1. check loop end
            if x_sto[0] >= destination_loc:
                break
            # 2. check light change
            if x_sto[-1] != x_sto[-2] and x_sto[0] < sto_opt.light.location:
                #### when light changed, switch to fixed pattern
                light_changed = True
                change_time = t

            # logic for the following
            # light_changed: 2a. next x after first light change captured -> generate fixed_policy
            #             2b. after this, fixed_policy is not empty, follow the fixed policy
            # light_changed not: follow mdp for new location gride

            if light_changed:

                if not fixed_policy_bool:
                    # print("captured change, ", x_sto, action_mdp)
                    if len(action_mdp) - 1 < x_sto[0]//sto_opt.dx:
                        # light change happens at the time when the change happens in the 
                        compansate_light -= 1
                    fixed_policy_bool = True

                    cur_light = x_sto[-2]
                    t0 = this_clock(cur_light, light) + t - change_time
                    opt_a, opt_plan, opt_vel = sto_opt.light_change_replan(x_sto[0], x_sto[1], t0, u)
                    # print(opt_plan, opt_a)
                    # analyze
                    if x_sto[0]//sto_opt.dx + 1 > len(action_mdp):
                        compansate_light = -1
                    
                    fixed_policy = opt_plan
                    action_mdp.append(opt_a)
                    # print("current state, ", x_sto, ", t = ", t)
                    # print("opt plan: ", opt_vel)
                    sto_opt.dynamics_change(x_sto, t)
                   

                elif x_sto[0] // sto_opt.dx + 2 + compansate_light > len(action_mdp):
                    if x_sto[0] >= sto_opt.light.location + sto_opt.dx:
                        break
                   
                    # follow the fixed_policy
                    new_act = fixed_policy.pop(0)
                    # print("state = ", x_sto)
                    # print(new_act)
                    action_mdp.append(new_act)
                    sto_opt.dynamics_change(x_sto, t)

            elif x_sto[0] // sto_opt.dx + 1 > len(action_mdp):
                # the vehicle goes to a new grid, we should change the policy
                # print(x_sto, action_mdp, t)
                if x_sto[0] < sto_opt.light.location + sto_opt.dx:
                    x = find_nearest(x_sto[0], sto_opt.dx)
                    v = find_nearest(x_sto[1], sto_opt.dv)

                    # vehicle does not reach the traffic light yet, action refer to traffic light
                    if np.isclose(x, light.location):
                        # find solution in fixed solution
                        
                        new_act = float(sto_opt.fixed_result[(sto_opt.fixed_result["x"]==x) & (sto_opt.fixed_result["v"]==v) & (sto_opt.fixed_result["init_light"]==init_light)]["plan"].values[0].strip('[]'))
                    else:
                        dT = 10000
                        index = -1
                        # print(t, x[0], x[1], sto_opt.mdp[sto_opt.mdp["x"] == x[0]][sto_opt.mdp["v"] == x[1]]["t"].values)
                        for i in range(len(sto_opt.mdp[(sto_opt.mdp["x"]==x) & (sto_opt.mdp["v"]==v) & (sto_opt.mdp["init_light"]==init_light)]["t"].values)):
                            t_ = sto_opt.mdp[(sto_opt.mdp["x"]==x) & (sto_opt.mdp["v"]==v) & (sto_opt.mdp["init_light"]==init_light)]["t"].values[i]
                            if abs(t - t_) < dT:
                                dT = abs(t - t_)
                                index = i
                                
                        new_act = sto_opt.mdp[(sto_opt.mdp["x"]==x) & (sto_opt.mdp["v"]==v) & (sto_opt.mdp["init_light"]==init_light)]["best_action"].values[index]
                   
                    new_act = sto_opt.adjust_acc(sto_opt.dx + x - x_sto[0], v, x_sto[1], new_act)
                    
                    action_mdp.append(new_act)
                    sto_opt.dynamics_change(x_sto, t)

                else:
                    # vehicle goes to the end of the road
                    # print(x_sto)
                    new_act = 'stop'
                    break
                    

            u = action_mdp[-1]

            # state function here
            if sto_opt.light.location - x_sto[0] < 0.5 and sto_opt.light.location > x_sto[0] and x_sto[1] < 1:
                
                if x_sto[2] == 3 and x_sto[-1] == 3:
                    # idling happens
                    idling_sto = True
                    x_sto[1] = 0
                    u = 'stop'
                    sto_opt.dynamics_change(x_sto, t)
                elif x_sto[2] == 1 and len(fixed_policy) > 0 and light_changed:
                    # change to green light, use next acc
                   
                    u = fixed_policy[0]
                    sto_opt.dynamics_change(x_sto, t)
                   
            cost_sto += cost_ins(u, x_sto[1], dt)
            x_sto = sto_opt.dynamics(x_sto, u, t)
            
            # update the light signal

            x_sto[2] = light.give_clock(t)
            traj_sto.append(x_sto[0])
            
            # cost_sto += cost_time_step(dt, u, w1, w2, w3, x_sto, light)
            time_prof_sto.append(t)
        


        # determined case
        x_det = copy.deepcopy(x0)
        cost_det = 0
        det_opt = dfs_optimizer(light, car)
        plan_det, cost, vel = det_opt.solver(x_det[0], x_det[1], dv)
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
            # if x_det[0] > 48 and x_det[0] < 51:
            #     print(x_det, u_det, t)
            cost_det += cost_ins(u_det, x_det[1], dt)    
            x_det = det_opt.dynamics(x_det, u_det, t)
            traj_det.append(x_det[0])
            time_prof_det.append(t)
        

        # dummy control
        dum_con = smart_control(light, car, a_max, a_min)
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

            ##############################
        


        # plt.rcParams.update({'font.size': 15})
        # plt.figure(figsize=(10, 8))

        # # plot the trajectory
        # # plt.plot(time_real, S[:, 0])


        # plt.plot(time_prof_sto, traj_sto, 'b')
        # plt.plot(time_prof_det, traj_det, 'k')
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

        # plt.title('Monte Carlo trail, t0 = '+str(t0_))
        # plt.grid(color='b', linestyle='-', linewidth=.2)
        # prefix = str(x0[1])
        # pic_name = 't0=' + str(t0_) + '_trail.png'
        # foldername = 'v0='+ str(x0[1])
        # plt.savefig(os.path.join(os.getcwd(), 'pics','Monte_Carlo', trafficFolderName, foldername, pic_name))
        # plt.close()
        print(t0_, 'saved!')
        comp = {
                "v0": x0[1],
                "t0": t0_,
                "det": cost_det,
                "sto": cost_sto,
                "dum": cost_dum
            }
        result = result.append(comp, ignore_index=True)


    
    filename = str(0) + '-' + str(x0[1]) + '-' + str(light.red_dur)+'-'+str(light.yel_dur)+'-'+str(light.gre_dur)+'_monte_carlo.csv'
    result.to_csv(os.path.join(os.getcwd(), 'data', filename), sep='\t')
    print('v0 = ', x0[1], ' csv result saved!')
    # cost_sto_plot = []
    # cost_det_plot = []
    # cost_dum_plot = []
    # for t0_ in t0_set:
    #     cost_sto_plot.append(result[result["t0"] == t0_]["sto"].values[0] * dt)
    #     cost_det_plot.append(result[result["t0"] == t0_]["det"].values[0] * dt)
    #     cost_dum_plot.append(result[result["t0"] == t0_]["dum"].values[0] * dt)
    # # print(cost_sto_plot)
    # plt.figure(figsize=(10, 8))

    # # plot the cost in one init vel
    # # plt.plot(time_real, S[:, 0])


    # plt.plot(t0_set, cost_sto_plot, 'b.')
    # plt.plot(t0_set, cost_det_plot, 'k*')
    # plt.plot(t0_set, cost_dum_plot, 'g^')
    # plt.legend(['sto', 'det', "smart"])
    # # plt.plot(time_real, vel, 'b')

    # plt.xlabel('t0/sec')
    # plt.ylabel('fuel cost/cm^3')
    
    # #plt.legend(['optimal trajectory', 'red', 'green', 'yellow'])

    # plt.title('Comparison of fuel cost, v0 = '+str(init_vel))
    # plt.grid(color='b', linestyle='-', linewidth=.2)
    # foldername = 'v0='+ str(x0[1])
    # pic_name = 'fuel_comp_v0='+ str(init_vel)+ '.png'
    # plt.savefig(os.path.join(os.getcwd(), 'pics', trafficFolderName, foldername, pic_name))


end_time = time.time()
total_time = (end_time - init_time)
ave_time = (end_time - init_time) / (len(vel_collection) * trail_num)
print("total time comsumed: %s, total trails num: %d, average time %s" %(total_time, len(vel_collection) * trail_num, ave_time))