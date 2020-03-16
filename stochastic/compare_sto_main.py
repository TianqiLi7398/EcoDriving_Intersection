from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
from sto_optimizer import stochastic_light, this_clock, next_clock, cost_time_step
from dum_controller import dummy_control
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import time
import pandas as pd
import copy

def newline(p1, p2, color):
    # draw a line from p1=[x,y] to p2
    ax = plt.gca()
    ymin, ymax = p1[1], p2[1]
    xmin, xmax = p1[0], p2[0]

    l = mlines.Line2D([xmin, xmax], [ymin, ymax], color=color)
    ax.add_line(l)
    return l

def find_nearest(x, n):
#     given x, find nearst integer times of n
    m = x // n

    if x % n > n/2:
        return n*(m + 1)
    else:
        return n*m


start_point = time.time()
x_init = 0
v_init = 18
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
w1 = 0.2   # work to do against friction
w2 = 0.1   # idling cost
w3 = 0.6   # accelerating cost weight
true_idling_cost = w2

t0_set = np.linspace(0, sum(timeset), sum(timeset)//2 + 1)



destination_loc = (m + 1) * delta_x + x_init

vel_collection = [5,10,15,20, 22]
# vel_collection = [15]


vel_collection = [10]
# t0_set = [28]


for init_vel in vel_collection:

    result = []

    for t0_ in t0_set:
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

        optimizer = dfs_optimizer(light, car)
        U, cost, vel = optimizer.solver(init_state["x"], init_state["v"], 1)


        sto_opt = stochastic_light(light, init_state, m, n, v_max, car, idling_cost)
        sto_opt.load_prior()

        dum_con = dummy_control(light, car)

        # cost_det_rec, cost_sto_rec = [], []
        ######################################################################
        # let's run the result of the mdp
        # plot the result trajectory
        dt = 0.01
        time_real = np.linspace(dt, light.T, 100*light.T)

        final_time = light.T
        # create the trajcetory


        fixed_policy = []

        # calculate the sto situation
        u = -1
        time_prof_sto = []
        traj_sto = []
        # vel_sto = []
        x_sto = x0
        # u_sto = []
        action_mdp = [sto_opt.mdp[sto_opt.mdp["x"] == x0[0]][sto_opt.mdp["v"] == x0[1]][sto_opt.mdp["init_light"] == init_light]["best_action"].values[0]]
        light_changed = False
        cost_sto = 0
        for t in time_real:
            # x: [x, v, l, lp]
            #   stochastic part
            ##check if the light changed
            if x_sto[0] > destination_loc:
                break;
            if x_sto[-1] != x_sto[-2] and x_sto[0] < sto_opt.light.location:
                #### when light changed, switch to fixed pattern
                light_changed = True
                change_time = t
                # print('light change observed before entering the traffic light location, the position is at x = ', x_sto[0], ', at t = ', t, ', new ligh is ', x_sto[-2])


        #     logic for the following
        #     light_changed: 1. next x after first light change captured -> generate fixed_policy
        #                 2. after this, fixed_policy is not empty, follow the fixed policy
        #     light_changed not: follow mdp for new location gride

            if x_sto[0] // sto_opt.dx + 1 > len(action_mdp):
                # the vehicle goes to a new grid, we should change the policy

                if x_sto[0] < sto_opt.light.location + sto_opt.dx:
                    x = find_nearest(x_sto[0], sto_opt.dx)
                    v = find_nearest(x_sto[1], sto_opt.dv)

                    if light_changed:


                        if len(fixed_policy) == 0:
                            # we should generate the fixed policy!
                            cur_light = x_sto[-2]
                            t0 = this_clock(cur_light, light) + t - change_time
                            opt_fix_light = traffic_light(t0, timeset, light.location)

                            x0_ = [x, v, opt_fix_light.give_clock(0)]
                            car_fix = opt_vehicle(sto_opt.dx, sto_opt.car.v_max, sto_opt.car.a_max, sto_opt.car.a_min, opt_fix_light, x0_)
                            optimizer_fix = dfs_optimizer(opt_fix_light, car_fix)
                            plan, cost, vel = optimizer_fix.solver(x, v, sto_opt.dv)
                            # print(plan, cost, vel, x_sto)
                            plan.reverse()
                            fixed_policy = plan
                            print('a, ',x_sto)
                            new_act = fixed_policy.pop()
                            action_mdp.append(new_act)
                        else:
                            print('b, ',x_sto)
        #                     follow the fixed_policy
                            new_act = fixed_policy.pop()
                            action_mdp.append(new_act)
                    else:
                        # vehicle does not reach the traffic light yet, and there is not
        #                 print(x_sto, t)
                        dT = 10000
                        index = -1
                        # print(t, x[0], x[1], sto_opt.mdp[sto_opt.mdp["x"] == x[0]][sto_opt.mdp["v"] == x[1]]["t"].values)
                        for i in range(len(sto_opt.mdp[sto_opt.mdp["x"] == x][sto_opt.mdp["v"] == v][sto_opt.mdp["init_light"] == init_light]["t"].values)):
                            t_ = sto_opt.mdp[sto_opt.mdp["x"] == x][sto_opt.mdp["v"] == v][sto_opt.mdp["init_light"] == init_light]["t"].values[i]
                            if abs(t - t_) < dT:
                                dT = abs(t - t_)
                                index = i
                                new_act = sto_opt.mdp[sto_opt.mdp["x"] == round(x_sto[0])][sto_opt.mdp["v"] == round(x_sto[1])][sto_opt.mdp["init_light"] == init_light]["best_action"].values[index]
                        action_mdp.append(new_act)

                else:
        #             vehicle goes to the end of the road
                    # print(x_sto)
                    new_act = 'end'
                    action_mdp.append(new_act)

            u = action_mdp[-1]

            #  state function here
            # old_x = copy.deepcopy(x_sto[0])

            if  sto_opt.light.location - x_sto[0] < 0.5 and x_sto[1] < 1:
                if x_sto[2] == 3 and x_sto[-1] == 3:
                    x_sto[1] = 0
                    u = 0
                    if not sto_opt.meet_red_inter:
                        # print('actually updated!!!!!!!!!!!!!!!')
                        action_mdp.append(sto_opt.meet_red_light())
                        sto_opt.meet_red_inter = True
                x_sto = sto_opt.dynamics(x_sto, u)

    #             if x_sto[-1] == 3 and x_sto[-2] == 1:

    #                 x_sto[0] = sto_opt.light.location + 0.01
        #             x_sto = sto_opt.dynamics(x_sto, u)
                if x_sto[0] > sto_opt.light.location:
                    x_sto = sto_opt.dynamics(x_sto, u)
            else:

                x_sto = sto_opt.dynamics(x_sto, u)
            # update the light signal

            x_sto[2] = light.give_clock(t)
            traj_sto.append(x_sto[0])
            # vel_sto.append(x_sto[1])
            # u_sto.append(u)
            cost_sto += cost_time_step(dt, u, w1, w2, w3, x_sto, light)
            # cost_sto_rec.append(cost_time_step(dt, x_sto[0] - old_x, u, w1, w2, a_max, v_max, true_idling_cost, x_sto, sto_opt.light, sto_opt.dx))
            time_prof_sto.append(t)


        u = -1
        traj_det = []
        # vel_det = []
        # u_det =  []
        cost_det = 0
        x_det = x0
        time_prof_det = []
        for t in time_real:
            # x: [x, v, l, lp]
            #   stochastic part
            ##check if the light changed
            if x_det[0] > destination_loc:
                break;
            ##############################
            #    fixed part

            # old_x = copy.deepcopy(x_det[0])

            # try:
            u = optimizer.controller(x_det)
            # except:
            #     print(vel_det)
            #     plt.plot(time_prof_det, vel_det)
            #     plt.show()

            x_det = optimizer.dynamics(x_det, u)



            x_det[2] = light.give_clock(t)
            traj_det.append(x_det[0])
            # vel_det.append(x_det[1])
            # u_det.append(u)
            # update the light signal
            cost_det += cost_time_step(dt, u, w1, w2, w3, x_sto, light)
            time_prof_det.append(t)
            # cost_det_rec.append(cost_time_step(dt, x_det[0] - old_x, u, w1, w2, a_max, v_max, true_idling_cost, x_det, sto_opt.light, sto_opt.dx))

            ##############################

        u = -1
        traj_dum = []
        # vel_dum = []
        # u_dum =  []
        cost_dum = 0
        x_dum = x0
        time_prof_dum = []
        for t in time_real:
            # x: [x, v, l, lp]
            #   stochastic part
            ##check if the light changed
            if x_dum[0] > destination_loc:
                break;
            ##############################
            #    fixed part

            old_x = copy.deepcopy(x_dum[0])

            u = dum_con.controller(x_dum)
            x_dum = dum_con.dynamics(x_dum, u)

            # update the light signal

            x_dum[2] = light.give_clock(t)
            traj_dum.append(x_dum[0])
            # vel_dum.append(x_dum[1])
            # u_dum.append(u)

            cost_dum += cost_time_step(dt, u, w1, w2, w3, x_sto, light)
            if x_dum[0] > dum_con.light.location and old_x < dum_con.light.location and x_dum[2] == 3:
                # print('running the light!!!')
                cost_dum += run_light_penalty
            time_prof_dum.append(t)

            ##############################


        print('****************************costs********')
        print('optimal: ', cost_det)
        print('MDP: ', cost_sto)
        print('Dum: ', cost_dum)

        plt.rcParams.update({'font.size': 15})
        plt.figure(figsize=(10, 8))

        # plot the trajectory
        # plt.plot(time_real, S[:, 0])


        plt.plot(time_prof_sto, traj_sto, 'b')
        plt.plot(time_prof_det, traj_det, 'k')
        plt.plot(time_prof_dum, traj_dum, 'g')
        plt.legend(['sto', 'det', 'dum'])
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
        # plt.ylim([0, light_location * 1.5])
        #plt.legend(['optimal trajectory', 'red', 'green', 'yellow'])

        plt.title('Comparison between planning on single vehicle deterministic and stochastic traffic light, t0 = '+str(t0_))
        plt.grid(color='b', linestyle='-', linewidth=.2)
        prefix = str(x0[1])
        pic_name = 'pics/v0='+ str(x0[1]) + '/t0=' + str(t0_) + '.png'
        plt.savefig(pic_name)
        print(t0_, 'saved!')

        comp = {
            "v0": x0[1],
            "t0:": t0_,
            "det_cost": cost_det,
            "sto_cost": cost_sto,
            "dum_cost": cost_dum
        }
        result.append(comp)


    df = pd.DataFrame(result)
    filename = 'data/' + str(0) + '-' + str(x0[1]) + '-' + str(light.red_dur)+'-'+str(light.yel_dur)+'-'+str(light.gre_dur)+'_result.csv'
    df.to_csv(filename, sep='\t')
    print('v0 = ', x0[1], ' csv result saved!')
