import copy
import math
import numpy as np
from util import opt_vehicle, traffic_light
from fuel_eco import cost_fuel
import os
import pandas as pd
from cost_table_gen import cost_fuel

class node_st:
    # this node stands for the grid states of (x,v) on the road
    def __init__(self, x, v, t):
        self.x = x
        self.v = v
        self.t = t
        self.income_edge = []
        self.cost = 9999999
        self.action = None

class state_space:
    '''
    a time-state searching method on deterministic part of traffic signal
    '''

    def __init__(self, light, car, dx = 0.25, dv = 1.0, dt = 0.5, extra_dist = 10.0):
        self.light = light
        # define the final time of searching the time 
        # of the beginning of next red light
        self.final_time = 2 * light.T - self.light.t0
        self.sol = []
        self.car = car
        self.graph = []
        self.optimal_control = None
        self.record = None
        self.stop_at_redlight = False
        self.go_after_redlight = False
        self.init_state = []
        # self.w1 = 0.2   # work to do against friction
        # self.w2 = 0.1   # idling cost
        # self.w3 = 0.6   # accelerating cost weight
        # self.alpha = 0.6289   # idling cost cm^3/s
        self.x0 = 0.0
        self.t0 = 0.0
        self.v0 = 0.0
        self.dx, self.dv, self.dt = dx, dv, dt
        self.m = round((self.light.location - self.x0 + extra_dist) / dx)
        self.n = round(self.car.v_max / dv) + 1
        self.N = int(round(self.final_time / dt))
        
        
        self.graph = {}
        self.x_terminate = self.m * self.dx
        
        self.solution = None
        # self.build_graph()
    
    def potential_states(self, x, v):
        v_map = np.linspace(0, self.car.v_max, self.n)
        
        delta_v = v_map - v
        aa = delta_v / self.dt
        
        # new_dv = np.delete(delta_v, np.bitwise_or(aa < self.car.a_min, aa > self.car.a_max))
        new_dv = delta_v[(aa >= self.car.a_min) & (aa <= self.car.a_max)]
        
        x_list, cost_list = [], []
        for dv in new_dv:
            a = dv / self.dt
            dx = v * self.dt + 0.5 * dv * self.dt
            x_list.append(x+dx)
            cost_list.append(cost_fuel(v, a, self.dt))
        return [x_list, new_dv+v, cost_list]
    
    def duration(self, v, a, dx):
        if np.isclose(dx, 0):
            return 0.0
        if abs(a) > 0:
            
            return (-v + np.sqrt(v**2 + 2 * a * dx)) / a
        else:
            return dx / v
    
    
    def validate_transition(self, x, v, t, new_x, new_v):
        
        if new_x < self.light.location or x > self.light.location:
            if new_v > 0:
                return True
            else:
                
                return False
        
        if new_v == 0.0:
            print(x, v, t, new_x, new_v)
        
        # all conditions that the transition is going through intersection line
        
        if self.light.give_clock(t) != 3:
            
            if self.light.give_clock(t + self.dt) != 3:
                print("1")
                return True
                # if np.isclose(new_v, 0) and not np.isclose(x, self.light.location):
                #     return False
                # else: return True
                # if new_v > 0:
                #     return True
                # else:
                #     return False
            # meets red light, check if it can pass 

            else:
                if np.isclose(new_v, 0.0) and np.isclose(new_x, self.light.location):
                    # choice of idling
                    print("2")
                    return True
                delta_x = self.light.location - x
                a = (new_v - v) / self.dt
                t_hit = self.duration(v, a, delta_x)
                assert t_hit <= self.dt, "t_hit greater than one time step"
                if self.light.give_clock(t + t_hit) != 3:
                    print("3")
                    return True
                else:
                    print("4")
                    return False
        
        else:
            
            # from red light to some light
            delta_x = self.light.location - x
            a = (new_v - v) / self.dt
            t_hit = self.duration(v, a, delta_x)
            
            assert t_hit <= self.dt, "t_hit greater than one time step"
            if self.light.give_clock(t + t_hit) == 1:
                # when pass the light, it is green light, okay just leave
                if new_v > 0:
                    print("5")
                    return True
                else:
                    print("6")
                    return False
            
            else:
                # red -> red, it must be stop at light.location with v=0
                if np.isclose(new_x, self.light.location) and np.isclose(new_v, 0.0):
                    print("7")
                    return True
                else:
                    print("8")
                    return False
    

    def validate_transition_atol(self, x, v, t, new_x, new_v, atol=1):
        
        if new_x < self.light.location - atol * self.dx or x > self.light.location:
            if new_v > 0:
                return True
            else:
                
                return False
        
        # if new_v == 0.0:
        #     print(x, v, t, new_x, new_v)
        
        # all conditions that the transition is going through intersection line
        
        if self.light.give_clock(t) != 3:
            
            if self.light.give_clock(t + self.dt) != 3:
                
                return True
                
            else:
                if np.isclose(new_x, self.light.location):
                    if np.isclose(new_v, 0.0):
                        return True
                    else:
                        return False
                
                elif new_x < self.light.location:
                    return True
                
                else:

                    delta_x = self.light.location - x
                    a = (new_v - v) / self.dt
                    t_hit = self.duration(v, a, delta_x)
                    assert t_hit <= self.dt, "t_hit greater than one time step"
                    if self.light.give_clock(t + t_hit) != 3:
                        
                        return True
                    else:
                        
                        return False
        
        else:

            if new_x < self.light.location:
                return True
            
            # from red light to some light
            delta_x = self.light.location - x
            a = (new_v - v) / self.dt
            t_hit = self.duration(v, a, delta_x)
            
            assert t_hit <= self.dt, "t_hit greater than one time step"
            if self.light.give_clock(t + t_hit) == 1:
                # when pass the light, it is green light, okay just leave
                if new_v > 0:
                    return True
                else:
                    
                    return False
            
            else:
                # red -> red, it must be stop at light.location with v=0
                if np.isclose(new_x, self.light.location) and np.isclose(new_v, 0.0):
                    
                    return True
                else:
                    
                    return False

    def update_terminate(self, x, v, h, new_v, key):
        delta_x = self.x_terminate - x
        a = (new_v - v) / self.dt
        t_hit = self.duration(v, a, delta_x)

        assert t_hit <= self.dt, "t_hit greater than one time step"
        new_cost = cost_fuel(v, a, t_hit) + h
        if self.graph[self.N + 1].cost > new_cost:
            self.graph[self.N + 1].cost = new_cost
            self.graph[self.N + 1].income_edge = key
            self.graph[self.N + 1].action = a
            self.graph[self.N + 1].v = v + a * t_hit
            self.graph[self.N + 1].t = key[-1] * self.dt + t_hit

        
    def build_graph(self):
        # time_map = np.linspace(self.dt, self.dt * self.N, self.N)
        # v_map = np.linspace(0, self.car.v_max, self.n)
        # x_map = np.linspace(self.dx, self.dx * self.m, self.m)
        self.graph = {}

        # initial state
        s0 = node_st(self.x0, self.v0, self.t0)
        self.graph[0] = s0
        sN = node_st(self.x_terminate, self.v0, self.t0)
        self.graph[self.N + 1] = sN

        # 1. initial state, find all potential states
        lattice = {}
        new_states = self.potential_states(self.x0, self.v0)
        # extend to next dt graph
        for i in range(len(new_states[0])):
            x = new_states[0][i]
            v = new_states[1][i]
            a = (v - self.v0) / self.dt
            new_node = node_st(x, v, self.dt)
            new_node.income_edge = [0, 0, 0]   # use x, v, t to determine the income edge
            new_node.cost = new_states[2][i]
            new_node.action = a
            lattice[int(round(x/self.dx)), int(round(v/self.dv))] = copy.deepcopy(new_node)

        self.graph[1] = copy.deepcopy(lattice)

        # propograte to all following lattices
        for i_t  in range(1, self.N + 1):
            t = self.dt * i_t
            lattice = {}
            for key in self.graph[i_t]:
                x, v = key[0] * self.dx, key[1] * self.dv
                new_states = self.potential_states(x, v)
                for i in range(len(new_states[0])):
                    new_x, new_v = new_states[0][i], new_states[1][i]
                    new_cost = new_states[2][i]
                    if self.validate_transition_atol(x, v, t, new_x, new_v):
                        a = (new_v - v) / self.dt
                        if new_x >= self.x_terminate:
                            self.update_terminate(x, v, self.graph[i_t][key].cost, new_v, [key[0], key[1], i_t])
                            continue
                        # build connection to new node
                        i_x, i_v = int(round(new_x/self.dx)), int(round(new_v/self.dv))
                        try:
                            if lattice[i_x, i_v].cost > self.graph[i_t][key].cost + new_cost:
                                lattice[i_x, i_v].cost = self.graph[i_t][key].cost + new_cost
                                lattice[i_x, i_v].income_edge = [key[0], key[1], i_t]
                                lattice[i_x, i_v].action = a
                        except:
                            new_node = node_st(new_x, new_v, t + self.dt)
                            new_node.income_edge = [key[0], key[1], i_t]   # use x, v, t to determine the income edge
                            new_node.cost = self.graph[i_t][key].cost + new_cost
                            new_node.action = a
                            lattice[i_x, i_v] = copy.deepcopy(new_node)
            
            if i_t < self.N:
                self.graph[i_t + 1] = copy.deepcopy(lattice)


    def forward_dynamic(self, x, v, a, dt):
        new_v = v + a * dt
        new_x = v * dt + .5 * a * dt**2 + x
        return [new_v, new_x]

    def dynamics(self, s, t, atol=1):
        #     state of the system
        #     s_k = (y, v, l, l_p)
        #     s_{k+1} = f(s_k, u_k)
        # check if idling, just stop the car:

        # open loop solution
        if t + self.car.delta_t >= self.solution["t"][0] and len(self.solution["t"]) > 1:
            self.solution["t"].pop(0)
            self.solution["a"].pop(0)
        if s[2] == 3 and np.isclose(s[0], self.light.location, atol = 0.1) and np.isclose(s[1], 0.0, atol=0.1):
            return [s[0], 0.0, s[2], s[2]], [0.0, 0.0]
        u = self.solution["a"][0]
        v0 = copy.deepcopy(s[1])
        
        location = s[0] + u * self.car.delta_t ** 2 / 2 + s[1] * self.car.delta_t
        vel = s[1] + u * self.car.delta_t
        if vel > self.car.v_max:
            
            print("alert for overspeeding!!!")
        lp = s[2]
        l = lp ### l will be updated by external light class
        return [location, vel, l, lp], [u, v0]
        
    
    def solver(self, x0, v0):
        # we need to know the initial position, then directly optimize it
        self.x0 = x0
        # self.t_base = 0
        self.v0 = v0
        
        self.build_graph()

        # based on the DP, trace back to find the solution
        solution = {"t": [], "a": [], "v": [], "x": []}
        solution["cost"] = self.graph[self.N + 1].cost
        solution["t"] = [self.graph[self.N + 1].t]
        solution["v"] = [self.graph[self.N + 1].v]
        solution["x"] = [self.graph[self.N + 1].x]
        solution["a"] = [self.graph[self.N + 1].action]
        i_x, i_v, i_t = self.graph[self.N + 1].income_edge

        while i_t > 0:
            solution["t"].insert(0, self.graph[i_t][i_x, i_v].t)
            solution["v"].insert(0, self.graph[i_t][i_x, i_v].v)
            solution["a"].insert(0, self.graph[i_t][i_x, i_v].action)
            solution["x"].insert(0, self.graph[i_t][i_x, i_v].x)
            i_x, i_v, i_t = self.graph[i_t][i_x, i_v].income_edge

        # validate the solution by forward kinematics, comment this out 
        # ONLY FOR DEBUG

        # solution["t"].insert(0, self.t0)
        # x_list, v_list = [], []
        # xk, vk = x0, v0
        # for i in range(len(solution["a"])):
        #     ak = solution["a"][i]
        #     dtk = solution["t"][i+1] - solution["t"][i]
        #     new_state = self.forward_dynamic(xk, vk, ak, dtk)
        #     x_list.append(new_state[1])
        #     v_list.append(new_state[0])
        #     xk = new_state[1]
        #     vk = new_state[0]
        
        self.solution = solution
        
        return solution