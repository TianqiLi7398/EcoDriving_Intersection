from util import opt_vehicle, traffic_light, vertex
from optimizer import dfs_optimizer
import numpy as np
import pandas as pd
import os
import time
import copy


def next_clock(init_light, light):
    # try to generate on state's optimal solution
    if init_light == 1:
        # green light, next light is yellow
        t0_opt = light.gre_dur
    elif init_light == 2:
        # yellow light, next light is red
        t0_opt = light.T - light.red_dur
    else:
        # red light, next light is green
        t0_opt = 0
    return t0_opt

def this_clock(init_light, light):
    # return the light's clock by current light observation
    if init_light == 1:
        # green light
        t0_opt = 0
    elif init_light == 2:
        # yellow light, next light is red
        t0_opt = light.gre_dur
    else:
        # red light, next light is green
        t0_opt = light.T - light.red_dur
    return t0_opt

def cost_time_step(dt, u, w1, w2, w3, s, light):
    # here delta_x is the grid x
    if u == 'stop':
        return dt * w2
    elif u == 'end':
        return 0
    elif u == 0 and light.location - s[0] < 0.5 and light.location - s[0] > 0 and abs(s[1]) < 0.1  and (s[2] == 3 or s[3] == 3):
        # red light, idling situation
        return dt * w2
    cost = w1 * dt
    if u > 0:
        # if it's accelerating, the work it does
        cost += w3 * u * (s[1] - u * dt / 2) * dt
    return cost

class stochastic_light:
    # this 'stochastic' here means the vehicle knows the light pattern (green, yellow, red)
    # duration of the signal, but the clock of the light is unkown
    def __init__(self, light, init_state, m, n, v_max, car, idling_cost):
        self.light = light
        self.init_state = init_state
        self.init_light = light.give_clock(0)
        self.light_generate = self.init_light
        self.next_light = self.init_light % 3 + 1
        self.car = car
        if self.init_light == 1:
            self.cur_period = self.light.gre_dur
        elif self.init_light == 2:
            self.cur_period = self.light.yel_dur
        else:
            self.cur_period = self.light.red_dur
        self.m = m
        self.n = n
        self.dx = (light.location - init_state["x"]) / m
        self.dv = v_max / (n - 1)
        self.loc_grid = np.linspace(self.dx + init_state["x"], light.location + self.dx, m + 1)
        self.vel_grid = np.linspace(0, v_max, n)
        self.nodes = pd.DataFrame(columns =["x", "v", "init_light", "t", "out_edge", "h", "must_happen"])  # stores all nodes when generating mdp, in self.expand()
        # filename = str(self.light.red_dur)+'-'+str(self.light.yel_dur)+'-'+str(self.light.gre_dur)+'.csv'

        filename = str(self.init_state["x"]) + '-' + str(self.init_state["v"]) + '-' + str(self.light.red_dur)+'-'+str(self.light.yel_dur)+'-'+str(self.light.gre_dur)+'.csv'
        filename = os.path.join(os.getcwd(),'data', filename)
        try:
            self.fixed_result = pd.read_csv(filename, sep='\t')
            print("deterministic file found!")
        except OSError as e:
            print("deterministic file not found!")
            self.generate_deter(filename)
            print("deterministic file generated!")
        self.car = car
        self.idling_cost = idling_cost
        self.meet_red_inter = False
        self.delta_t_min = self.dx / v_max
        self.w1 = 0.2   # work to do against friction
        self.w2 = 0.1   # idling cost
        self.w3 = 0.6   # accelerating cost weight
        self.alpha = 0.6289    # the stop-and-go cost cm^3/s
        self.x_base = self.init_state["x"]
        self.t_base = 0
        self.v_base = self.init_state["v"]
        try:
            # read csv file
            self.cost_table = pd.read_csv(os.path.join(os.getcwd(), 'data', 'cost_fix_dis.csv'))    
        except:
            print("no cost table file found! ")

    def fuel_cost(self, a, v, delta_t, delta_v):

        # cost = self.w1 * delta_t  

        # if a > 0:
        #     # if it's accelerating, the work it does
        #     cost = self.w3 * a * (v + delta_v / 2) * delta_t
        
        cost = self.cost_table[(self.cost_table["v"] == int(v)) & (self.cost_table["dv"] == int(delta_v))]["cost"].values[0]

        return cost

    def get_cur_period(self):
        if self.light_generate == 1:
            self.cur_period = self.light.gre_dur
        elif self.light_generate == 2:
            self.cur_period = self.light.yel_dur
        else:
            self.cur_period = self.light.red_dur

    def unchange_prob_uni(self, state, delta_t):
        # uniform distribution
        return 1 - self.change_prob(state, delta_t)

    def change_prob_uni(self, ver, delta_t):
        # uniform distribution
        prob = delta_t/(self.cur_period - ver["t"])
        if prob < 0:
            prob = 1
        return min(prob, 1)

    def change_prob_uni_df(self, index, delta_t):
        prob = delta_t/(self.cur_period - self.node_frame["t"][index])
        if prob < 0:
            prob = 1
        return prob

    def unchange_prob_uni_df(self, index, delta_t):
        # uniform distribution
        return 1 - self.change_prob_df(index, delta_t)

    def create_descendent(self, ver, x, v):
        delta_v = v - ver["v"]
        
        delta_t = 2 * self.dx/(v + ver["v"])
        a = delta_v / delta_t
        if a <= self.car.a_max and a >= self.car.a_min:
            delta_t = round(delta_t, 2)
            t = delta_t + ver["t"]
            # dynamic check
            # j = delta_t * self.w1 / self.delta_t_min + abs(a/self.car.a_max) * self.w2
            j = self.fuel_cost(a, ver["v"], delta_t, delta_v)

            prob = self.change_prob_uni(ver, delta_t)
            ver["out_edge"].append([x, v, delta_t, j, a, prob, self.light_generate])

            if len(self.nodes[(self.nodes["t"] == t) & (self.nodes["x"] == x) & (self.nodes["init_light"] == self.light_generate) & (self.nodes["v"] == v)].index) > 0:
                # if the descendent is created, just point the ver to its descendent
                # print("So this may be true!!!!")
                return

            if np.isclose(prob, 1):
                new_ver = {
                    "x": x,
                    "v": v,
                    "init_light": self.light_generate,
                    "t": t,
                    "out_edge": [],
                    "h": 99999,
                    "must_happen": 1}
            else:
                new_ver = {
                    "x": x,
                    "v": v,
                    "init_light": self.light_generate,
                    "t": t,
                    "out_edge": [],
                    "h": 99999,
                    "must_happen": -1}

            # new_ver["in_edge"].append([x, v, delta_t, j, a, prob, light])
            #   end expand condition:
            # 1. expand till the intersection location
            # 2. prob of change of one next state is 1, this must happen
            if np.isclose(x, self.loc_grid[-2]):
                new_ver["h"] = float(self.fixed_result[(self.fixed_result["init_light"] == self.light_generate) & (self.fixed_result["x"] == x) & (self.fixed_result["v"] == v)]["cost"].values[0])      
                # print(new_ver["h"])
                # reward for passing the intersection, -100
                self.nodes = self.nodes.append(new_ver, ignore_index=True)
                return
            elif new_ver["must_happen"] > 0:
                # it is going to happen, no descendent
                self.nodes = self.nodes.append(new_ver, ignore_index=True)
                return
            else:
                self.nodes = self.nodes.append(new_ver, ignore_index=True)
                self.expand(self.nodes.loc[len(self.nodes) - 1])

    def expand(self, ver):
        # in python, the address is assigned when you assign one variable
        # to another, so when new vertex is created, as long as it is saved
        # in edge, it's fine
        # some logic:
        # a. if the start light is red (3), at intersection, v>0 is not allowed; before and after intersection, v = 0 is not allowed
        # b. if start light is other, v = 0 is not allowed all spaces
        x = ver["x"] + self.dx
        if x > self.loc_grid[-1]:
            return

        if self.light_generate == 3:
            # red light, for safety, assume next state you can only stop
            if np.isclose(x, self.light.location):
                self.create_descendent(ver, x, 0)
            else:
                for v in self.vel_grid[1:]:
                    # no v = 0
                    self.create_descendent(ver, x, v)
        else:
            # other light, need to consider the case that light will change within the travel of last dx to intersection
            if np.isclose(x, self.light.location):
                for v in self.vel_grid:
                    # need to consider if you need to stop at red light
                    self.create_descendent(ver, x, v)
            else:
                for v in self.vel_grid[1:]:
                    # do not need to consider if you need to stop, that is velocity cannot be 0
                    self.create_descendent(ver, x, v)

    def cal_idle(self, t, dt, light):
        # calculate the expectation of idling cost in mdp transition. 
        if light == 1:
            if dt < self.light.yel_dur:
                p = 0
                time = 0
            else:
                p = (dt - self.light.yel_dur) / (self.light.gre_dur - t)
                time = (self.light.red_dur - dt + self.light.yel_dur + self.light.red_dur) * 0.5
        elif light == 2:
            if t + dt < self.light.yel_dur:
                p = dt / self.light.yel_dur
                time = (self.light.red_dur + self.light.red_dur - dt) * 0.5
            elif t + dt < self.light.yel_dur + self.light.red_dur:
                p = 1
                time = (self.light.red_dur - (t + dt - self.light.yel_dur) + self.light.red_dur - dt) * 0.5
            else:
                p = 0
                time = 0
        else:
            if t + dt < self.light.red_dur:
                p = (self.light.red_dur - t - dt) / self.light.red_dur
                time = (self.light.red_dur - (t + dt)) * 0.5
            else:
                p = 0
                time = 0

        # return p*time*self.idling_cost
        return p * time * self.alpha 

    def backtrack(self):
        # generate the MDP model, which comes from the last column till the first based on Bellman equation
        print('get into back track func')
        mdp = []
        # 1. first, transfer list to a dataframe
        self.node_frame = pd.DataFrame(self.nodes)
        print('intotal ', len(self.nodes), ' nodes for MDP!')
        # 2. second, from the second last column till the first, generate the utility func for each node by Bellman's equation
        # h_1(s) = min_a {p(light change, s1| a, s)*h_2(s1) + p(light not change, s2| a, s)*h_2(s2)

        for i in range(self.m):
            x = self.loc_grid[self.m - i - 1] - self.dx
            print('back track x = ', x)
            for j in self.node_frame[self.node_frame["x"] == x].index:
                best_index = 0
                edges = self.node_frame["out_edge"][j]
                t = self.node_frame["t"][j]
                if len(edges) > 0:
                    # edge: [x, v, delta_t, j, a, change_prob, light]
                    # print(self.node_frame[self.node_frame["init_light"] == self.init_light][self.node_frame["x"] == edges[0][0]]["h"].values)
                    if np.isclose(x, self.light.location - self.dx):
                        prob_change = edges[0][-2]
                        light = edges[0][-1]
                        next_light = light % 3 + 1
                        idle_cost = self.cal_idle(t, edges[0][2], light)
                        t_plus = t + edges[0][2]
                        min_h = prob_change * float(self.fixed_result[(self.fixed_result["init_light"]==next_light) & (self.fixed_result["x"]==edges[0][0]) & (self.fixed_result["v"]==edges[0][1])]["cost"].values[0]) + \
                                (1 - prob_change) * self.node_frame[(self.node_frame["x"]==edges[0][0]) & (self.node_frame["v"]==edges[0][1]) & (self.node_frame["init_light"]==light) & (self.node_frame["t"]==t_plus)]["h"].values[0] + \
                                edges[0][3] + idle_cost
                        if len(edges) > 1:
                            for k in range(1, len(edges)):
                                t_plus = t + edges[k][2]
                                idle_cost = self.cal_idle(t, edges[k][2], light)
                                prob_change = edges[k][-2]
                                h = prob_change * float(self.fixed_result[(self.fixed_result["init_light"]==next_light) & (self.fixed_result["x"]==edges[k][0]) & (self.fixed_result["v"]==edges[k][1])]["cost"].values[0]) + \
                                    (1 - prob_change) * self.node_frame[(self.node_frame["x"]==edges[k][0]) & (self.node_frame["v"]==edges[k][1]) & (self.node_frame["init_light"] == light) & (self.node_frame["t"] == t_plus)]["h"].values[0] + \
                                    edges[k][3] + idle_cost
                                if h < min_h:
                                    best_index = k
                                    min_h = h
                    else:
                        prob_change = edges[0][-2]
                        light = edges[0][-1]
                        next_light = light % 3 + 1
                        t_plus = t + edges[0][2]
                        min_h = prob_change * float(self.fixed_result[(self.fixed_result["init_light"]==next_light) & (self.fixed_result["x"]==edges[0][0]) & (self.fixed_result["v"]==edges[0][1])]["cost"].values[0]) + \
                                (1 - prob_change) * self.node_frame[(self.node_frame["x"]==edges[0][0]) & (self.node_frame["v"]==edges[0][1]) & (self.node_frame["init_light"]==light) & (self.node_frame["t"]==t_plus)]["h"].values[0] + \
                                edges[0][3]
                        if len(edges) > 1:
                            for k in range(1, len(edges)):
                                t_plus = t + edges[k][2]
                                prob_change = edges[k][-2]
                                h = prob_change * float(self.fixed_result[(self.fixed_result["init_light"]==next_light) & (self.fixed_result["x"]==edges[k][0]) & (self.fixed_result["v"]==edges[k][1])]["cost"].values[0]) + \
                                    (1 - prob_change) * self.node_frame[(self.node_frame["x"]==edges[k][0]) & (self.node_frame["v"]==edges[k][1]) & (self.node_frame["init_light"]==light) & (self.node_frame["t"]==t_plus)]["h"].values[0] + \
                                    edges[k][3]
                                if h < min_h:
                                    best_index = k
                                    min_h = h

                    self.node_frame["h"][j] = min_h
                    # save the result
                    state = {
                        "x" : x,
                        "v" : self.node_frame["v"][j],
                        "t" : t,
                        "best_action" : edges[best_index][-3],
                        "h" : min_h,
                        "prob_change": edges[best_index][-2],
                        "init_light": light
                    }
                    mdp.append(state)

        # then we save the result
        self.mdp_result = pd.DataFrame(mdp)
        filename = str(self.init_state["x"]) + '-' + str(self.init_state["v"]) + '-' + str(self.light.red_dur)+'-'+str(self.light.yel_dur)+'-'+str(self.light.gre_dur)+'_sto.csv'
        filename = filename = os.path.join(os.getcwd(),'data', filename)
        self.mdp_result.to_csv(filename, sep='\t')
        # for i in self.node_frame[self.node_frame["x"] == self.dx].index:
        #     delta_v = self.node_frame["v"][i] - self.init_state["v"]
        #     delta_t = 2 * self.dx / (self.node_frame["v"][i] + self.init_state["v"])
        #     a = delta_v / delta_t
        #     if a <= self.car.a_max and a >= self.car.a_min:
        #         j = delta_t * self.car.w1 / self.car.delta_t_min + abs(a/self.car.a_max) * self.car.w2
    
    def load_prior(self):
        filename = str(self.init_state["x"]) + '-' + str(self.init_state["v"]) + '-' + str(self.light.red_dur)+'-'+str(self.light.yel_dur)+'-'+str(self.light.gre_dur)+'_sto.csv'
        filename = os.path.join(os.getcwd(),'data', filename)
        print("file to be opened "+ filename)
        try:
            self.mdp = pd.read_csv(filename, sep = '\t')
            print("file opened!")
        except OSError as e:
            print(filename + " doesn't exists in the local path!")
            #  calculate the time for generating
            a = time.time()
            three_init_time = [a]

            for init_light in [1,2,3]:
                print("generate light for light = ", init_light)
                state = copy.deepcopy(self.init_state)
                state["init_light"] = init_light
                self.nodes = self.nodes.append(state, ignore_index=True)
                #   self.nodes[-1]["init_light"] = init_light
                self.light_generate = init_light
                self.get_cur_period()
                
                self.expand(self.nodes.loc[len(self.nodes) - 1])
                # self.expand(state)
                b = time.time()
                three_init_time.append(b)
                print('time spent for light ', init_light, ' , is ', three_init_time[init_light] - three_init_time[init_light-1])
            print('get out of the loop!')
            self.backtrack()
            c = time.time()
            print('time spend for MDP generating is ', c - three_init_time[-1])
            print(filename + " exists now!")
            self.mdp = self.mdp_result


    def controller(self, x):
        pass
        # if self.go_after_redlight:
        #     if x[0] < self.light.location + self.car.delta_x:
        #         return self.optimal_control[-1]
        # if x[0] < self.light.location:
        #     if x[2] != 3:
        #         # if x[3] == 3 and self.light.location - x[0] < 0.5 and x[1] == 0:
        #         if x[3] == 3 and self.stop_at_redlight:
        #             self.go_after_redlight = True
        #             return self.optimal_control[-1]
        #         index = math.floor((x[0] - self.car.x0[0])/self.car.delta_x)
        #         # print(x[0], self.car.x0[0], self.car.delta_x, index)
        #         return self.optimal_control[index] + self.feedback_controller(x, index)
        #     elif x[3] == 3 and self.light.location - x[0] < 0.5 and abs(x[1]) < 0.5:
        #         self.stop_at_redlight = True
        #         return 'stop'
        #     else:
        #         index = math.floor((x[0] - self.car.x0[0])/self.car.delta_x)
        #         # print(x[0], self.car.x0[0], self.car.delta_x, index)
        #         return self.optimal_control[index] + self.feedback_controller(x, index)

        # else:
        #     return 0

    def dynamics(self, s, u, t):
        #     state of the system
        #     s_k = (y, v, l, l_p)
        #     s_{k+1} = f(s_k, u_k)

        if u == 'stop':
            s[1] = 0
            
        else:
            dt = t - self.t_base
            s[0] = ((u * (dt ** 2)) / 2) + (self.v_base * dt) + self.x_base
            s[1] = u * dt + self.v_base
        s[-1] = s[2]
        ### l will be updated by external light class
        return s
    
    def dynamics_change(self, s, t):
        self.t_base = t - self.car.delta_t
        self.v_base = s[1]
        self.x_base = s[0]
        return

    def generate_deter(self, filename):
        timeset = [self.light.red_dur, self.light.yel_dur, self.light.gre_dur]
        # loc_grid = np.linspace(dx, light_location + dx, m + 1)
        # vel_grid = np.linspace(0, v_max, n)
        fix_sols = []
        for init_light in [1, 2, 3]:

            opt_light = traffic_light(this_clock(init_light, self.light), timeset, self.light.location)
            for v in self.vel_grid:
                for x in self.loc_grid[:-1]:
                    if np.isclose(v, 0) and x < self.light.location:
                        continue
                    x0_ = [x, v, opt_light.give_clock(0)]
                    car = opt_vehicle(self.dx, self.car.v_max, self.car.a_max, self.car.a_min, opt_light, x0_, self.car.delta_t)
                    optimizer = dfs_optimizer(opt_light, car)
                    plan, cost, vel = optimizer.solver(x, v, self.dv)
                    item = {"x": x,
                           "v": v,
                           "cost": cost,
                           "plan": plan,
                           "vel": vel,
                           "init_light": init_light}
                    fix_sols.append(item)
        self.fixed_result = pd.DataFrame(fix_sols)
        # save the file for future use       
        self.fixed_result.to_csv(filename, sep='\t')

    def meet_red_light(self):
        a = self.fixed_result[(self.fixed_result["init_light"]==1) & (self.fixed_result["x"]==self.m * self.dx) & (self.fixed_result["v"]==0)]["plan"].values[0]
        b = a[1:-1]
        return float(b)
    
    def replan(self, dx, dv, v):
        
        dt = dx / (v + dv * 0.5)
        a = dv / dt
        if a > self.car.a_max:
            a = self.car.a_max
        elif a < self.car.a_min:
            a = self.car.a_min
        return a

    def adjust_acc(self, x, v_bar, v, a):
        # v_bar: cloest v at this grid,
        # v: actual v
        if np.isclose(a, 0):
            v_aim = v_bar
        else:
            delta = np.sqrt(v_bar**2 + 2 * a * self.dx)
            dt = (delta - v_bar)/a
            v_aim = v_bar + a * dt
        dv = v_aim - v
        return self.replan(x, dv, v)
    
    def cost_acum(self, v, a, delta_t):
        dt = 0.01
        
        if delta_t < dt:
            return self.inst_fuel_cost(v, a) * delta_t
        length = int(delta_t//dt)
        # print(length)
        # time = np.linspace(dt, length *dt, length)
        cost = 0
        for i in range(length):
            cost += self.inst_fuel_cost(v, a) * dt
            v += a * dt
        return cost
    
    def inst_fuel_cost(self, v, a):
        # Speed v: m/s
        # acceleration a: m/s^2
        # output: originally is L/s, change it to cm^3/s for computation convinence 
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
        alpha0 = 0.0006289
        alpha1 = 2.676e-05
        alpha2 = 1e-06
        P =float(((R + 1.04 * m * acc) / (3600 * etaD)) * vel)
        #alpha0 = float((Pmf0 * widle * d * 3.42498) / (22164 * Q * N))

        Fuel_Consumption =(alpha0 + alpha1 * P + alpha2 * (P *P))
        if P < 0:
            Fuel_Consumption = float(alpha0)
        #print(alpha0,alpha1,alpha2)
        return Fuel_Consumption * 1000
    
    def light_change_replan(self, x, v, t0, a0):
        x_next = (x // self.dx + 1) * self.dx
        # print("x_next = ", x_next)
        dx = x_next - x
        cost = 99999
        if t0 < 0:
            # no light change observed
            # means this is just calculate the last step through intersection
            dx = self.light.location + self.dx - x 
            for v_next in self.vel_grid[1:]:
                dv = v_next - v
                # print(v_next, dv, v)
                delta_t = 2 * dx/(v_next + v)
                a = dv / delta_t
                cost1 = self.cost_acum(v, a, delta_t)
                if cost1 < cost:
                    cost = cost1 
                    opt_a = a
            return opt_a

        light = copy.deepcopy(self.light)
        
        opt_a = -100
        if v > 0:
            if np.isclose(x_next, self.light.location):
                for v_next in self.vel_grid:
                    dv = v_next - v
                    # print(v_next, dv, v)
                    delta_t = 2 * dx/(v_next + v)
                    
                    a = dv / delta_t
                    cost1 = self.cost_acum(v, a, delta_t)
                    if a <= self.car.a_max and a >= self.car.a_min:
                        t = t0 + delta_t
                        light.t0 = t 
                        # x0 = [x_next, v_next, light.give_clock(0)]
                        optimizer_fix = dfs_optimizer(light, self.car)
                        plan, cost2, vel = optimizer_fix.solver(x_next, v_next, self.dv)
                        if cost > cost1 + cost2:
                            opt_a = a
                            opt_plan = plan 
                            opt_vel = vel
                            # print("one iteras:")
                            # print(opt_a, opt_vel)
                            cost = cost1 + cost2
                if opt_a == -100:
                    opt_a = a0
                    for v_next in self.vel_grid[1:]:
                        dv = v_next - 0
                        # print(v_next, dv, v)
                        delta_t = 2 * self.dx/(v_next + v)
                        a = dv / delta_t
                        cost1 = self.cost_acum(0, a, delta_t)
                        if cost1 < cost:
                            cost = cost1 
                            opt_plan = [a]
                            opt_vel = [0, v_next]
                    return opt_a, opt_plan, opt_vel
            else:
                for v_next in self.vel_grid[1:]:
                    dv = v_next - v
                    # print(v_next, dv, v)
                    delta_t = 2 * dx/(v_next + v)
                    
                    a = dv / delta_t
                    
                    cost1 = self.cost_acum(v, a, delta_t)
                    if a <= self.car.a_max and a >= self.car.a_min - 0.2:
                        t = t0 + delta_t
                        light.t0 = t 
                        # x0 = [x_next, v_next, light.give_clock(0)]
                        optimizer_fix = dfs_optimizer(light, self.car)
                        plan, cost2, vel = optimizer_fix.solver(x_next, v_next, self.dv)
                        
                        if cost > cost1 + cost2:
                            opt_a = max(a, self.car.a_min)
                            opt_plan = plan 
                            opt_vel = vel
                            # print("one iteras:")
                            # print(opt_a, opt_vel)
                            cost = cost1 + cost2
                
        else:
            t = t0 
            light.t0 = t 
            # x0 = [x_next, v, light.give_clock(0)]
            optimizer_fix = dfs_optimizer(light, self.car)
            opt_plan, cost2, opt_vel = optimizer_fix.solver(x_next, v, self.dv)
            opt_a = opt_plan[0]        
        
        if opt_a == -100:
            opt_a = 0
            delta_t = dx / v            
            t = t0 + delta_t
            light.t0 = t 
            # x0 = [x_next, v, light.give_clock(0)]
            optimizer_fix = dfs_optimizer(light, self.car)
            opt_plan, cost2, opt_vel = optimizer_fix.solver(x_next, v, self.dv)
        # print(opt_a)
        return opt_a, opt_plan, opt_vel