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
    # instanuous fuel cost
    # basic cost
    w1 = 0.2   # work to do against friction
    w2 = 0.1   # idling cost
    w3 = 0.6   # accelerating cost weight
    if a == 'stop':
        # idling cost
        cost = dt * w2
    elif a > 0:
        # if it's accelerating, the work it does
        cost = w3 * a * (v + 0.5 * a * dt) * dt
    else:
        # cost for breaking or no acc
        cost = w1 * dt

    return cost

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
vel_collection = [15]
# t0_set = [28]


for init_vel in vel_collection:

    result = pd.DataFrame(columns=["v0", "t0", "det", "sto", "dum"])

    for t0_ in t0_set:
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

        # stochastic case
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
                    
                    # x = find_nearest(x_sto[0], sto_opt.dx)
                    
                    # # we should generate the fixed policy!
                    # opt_fix_light = traffic_light(t0, timeset, light.location)
                    # if np.isclose(x, sto_opt.light.location) and x_sto[2] == 3:
                    #     v = 0
                    # x0_ = [x, v, opt_fix_light.give_clock(0)]
                    # car_fix = opt_vehicle(sto_opt.dx, sto_opt.car.v_max, sto_opt.car.a_max, sto_opt.car.a_min, opt_fix_light, x0_, dt)
                    # optimizer_fix = dfs_optimizer(opt_fix_light, car_fix)
                    # plan, cost, vel = optimizer_fix.solver(x, v, sto_opt.dv)
                    # # print(plan, cost, vel, x_sto)
                    # plan.reverse()
                    # fixed_policy = plan
                   
                    # # replan to compensate the approximate error for the first fixed policy
                    # if x > x_sto[0]:
                    #     dis_x = x - x_sto[0]
                    #     dis_v = vel[0] - x_sto[1]
                    # else:
                    #     dis_x = x + sto_opt.dx - x_sto[0] 
                    #     dis_v = vel[1] - x_sto[1]
                    #     # if compansate_light > -1:
                    #     new_act = fixed_policy.pop()
                    # # if np.isclose(v, 0) and np.isclose(dis_v, 0) and dis_x < 1:
                    # if np.isclose(x, sto_opt.light.location) and abs(dis_v) < 1:
                    #     # no need to replan, this is actually restart for green light
                    #     new_act = fixed_policy.pop()
                    #     compansate_light -= 1
                    # else:
                    #     new_act = sto_opt.replan(dis_x, dis_v, x_sto[1])
                    # # print("replan: dx: %s, dv = %s, a = %s" % (dis_x, dis_v, new_act))
                    # # print(x_sto[0] // sto_opt.dx + compansate_light, len(action_mdp))
                    # action_mdp.append(new_act)
                    # sto_opt.dynamics_change(x_sto, t)

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
                        # print(x, v, x_sto)
                        # opt_a = sto_opt.light_change_replan(x_sto[0], x_sto[1], -1)
                        # print(opt_a)
                        # new_act = opt_a
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
                    # action_mdp.append(new_act)

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
                    # if not sto_opt.meet_red_inter:
                    #     # print('actually updated!!!!!!!!!!!!!!!')
                    #     action_mdp.append(sto_opt.meet_red_light())
                    #     sto_opt.meet_red_inter = True
            # if x_sto[0] > 20 and x_sto[0] < 30:
            #     print('state = ', x_sto, ' , acc = ', u, 'len = ', len(action_mdp))
            cost_sto += cost_ins(u, x_sto[1], dt)
            x_sto = sto_opt.dynamics(x_sto, u, t)
            
            # update the light signal

            x_sto[2] = light.give_clock(t)
            traj_sto.append(x_sto[0])
            # vel_sto.append(x_sto[1])
            # u_sto.append(u)
            cost_sto += cost_time_step(dt, u, w1, w2, w3, x_sto, light)
            # cost_sto_rec.append(cost_time_step(dt, x_sto[0] - old_x, u, w1, w2, a_max, v_max, true_idling_cost, x_sto, sto_opt.light, sto_opt.dx))
            time_prof_sto.append(t)
        if idling_sto:
            cost_sto += alpha

        

        plt.rcParams.update({'font.size': 15})
        plt.figure(figsize=(10, 8))

        # plot the trajectory
        # plt.plot(time_real, S[:, 0])


        plt.plot(time_prof_sto, traj_sto, 'b')
        # plt.plot(time_prof_det, traj_det, 'k')
        # plt.plot(time_prof_dum, traj_dum, 'g')
        plt.legend(['sto', 'det', "smart"])
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
        pic_name = 't0=' + str(t0_) + '_sto.png'
        foldername = 'v0='+ str(x0[1])
        plt.savefig(os.path.join(os.getcwd(), 'pics', foldername, pic_name))
        print(t0_, 'saved!')
    
        print(cost_sto)


    
    # filename = str(0) + '-' + str(x0[1]) + '-' + str(light.red_dur)+'-'+str(light.yel_dur)+'-'+str(light.gre_dur)+'_result.csv'
    # result.to_csv(os.path.join(os.getcwd(), 'data', filename), sep='\t')
    # print('v0 = ', x0[1], ' csv result saved!')

