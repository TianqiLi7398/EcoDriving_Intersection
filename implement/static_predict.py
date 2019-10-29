import copy
import math
import numpy as np
from util import opt_vehicle, traffic_light


def Sort_Tuple(tup):

    # reverse = None (Sorts in Ascending order)
    # key is set to sort using second element of
    # sublist lambda has been used
    tup.sort(key=lambda x: x[0])
    return tup


class pre_optimizer:
    def __init__(self, light, car):
        self.light = light
        final_time = 2 * light.T
        self.red, self.yellow, self.green = light.trafficline(final_time)
        self.sol = []
        self.car = car
        self.graph = []
        self.optimal_control = None
        self.record = None
        self.stop_at_redlight = False
        self.go_after_redlight = False
        self.penalty_red = 1

    class node:
        # this node stands for the grid states of (x,v) on the road
        def __init__(self, x, v):
            self.x = x
            self.v = v
            self.income_edge = []
            self.outcome_edge = []
            self.cost = 99999
            self.action = None

    def inside_red(self, t):
        # to check when vehicle arrives at the intersection, if the signal is red
        # for time_area in self.red:
        #     if t > time_area[0] and t < time_area[1]:
        #         return [True, time_area[1] - t]
        # return [False, None]
        for i in range(int(len(self.red)/2)):
            if t > self.red[2*i][0] and t < self.red[2*i + 1][0]:
                return [True, self.red[2*i + 1][0] - t]
        return [False, None]

    def DFS_green_light(self, node, sol):
        if node.x < self.car.x0[0] + self.car.delta_x:
            # in prediction, we need to make penalty if real light is not green.
            #  find if the total time is inside the green/red interval
            p_red = self.light.prediction(self.car.x0[2], sol["t"])[0]
            #  add the solution to the whole solution
            cost = sol["cost"] + self.penalty_red * abs(max(np.log(p_red), -9999))
            print(cost)
            self.sol.append([cost, sol["action"]])
            return
        else:
            if node.income_edge != []:

                for edge in node.income_edge:
                    #                     new_sol = new_sol
                    new_sol = copy.deepcopy(sol)
                    new_sol["t"] += edge["delta_t"]
                    new_sol["cost"] += edge["cost"]
                    new_sol["action"].insert(0, edge["a"])
#                     print(edge["v1"].x, edge["v1"].v)
                    self.DFS_green_light(edge["v1"], new_sol)

    def DFS_red_light(self, node, sol):
        if node.x < self.car.x0[0] + self.car.delta_x:
            # meet_red_light = self.inside_red(sol["t"])
            #  find if the total time is inside the green/red interval
            # if meet_red_light[0]:
            #     #  add the solution to the whole solution
            #     idling_cost = meet_red_light[1] * 1
            idling_cost = self.light.est_waiting(self.car.x0[2], sol["t"]) * 1
            if idling_cost > 0:
                self.sol.append([sol["cost"] + idling_cost, sol["action"]])
            return
        else:
            if node.income_edge != []:
                # solution exists
                for edge in node.income_edge:
                    new_sol = copy.deepcopy(sol)
                    new_sol["t"] += edge["delta_t"]
                    new_sol["cost"] += edge["cost"]
                    new_sol["action"].insert(0, edge["a"])
#                     print(edge["v1"].x, edge["v1"].v)
                    self.DFS_red_light(edge["v1"], new_sol)

    def build_graph(self, loc_grid, vel_grid):

        for loc in loc_grid:
            row = []
            for vel in vel_grid:
                row.append(self.node(loc, vel))
            self.graph.append(row)
        begin_node = self.node(self.car.x0[0], self.car.x0[1])
        final_node = self.node(loc_grid[-1], 0)

        # first layer of nodes initialize
        for node in self.graph[0]:
            delta_v = node.v - begin_node.v
            delta_t = 2 * self.car.delta_x/(node.v + begin_node.v)
            a = delta_v / delta_t
            if a < self.car.a_max and a > self.car.a_min:

                j = delta_t * self.car.w1 / \
                    self.car.delta_t_min + abs(a/self.car.a_max)*self.car.w2
                edge = {"v1": begin_node, "v2": node, "delta_t": delta_t, "cost": j, "a": a}
                node.income_edge.append(edge)
                begin_node.outcome_edge.append(edge)

        # later layes
        # build the graph
        for i in range(len(loc_grid)):
            if loc_grid[i] <= self.light.location:
                # calculate all edges before the intersection
                for j in range(len(self.graph[i+1])):
                    if self.graph[i][j].income_edge != []:
                        # means this edge is reachable from initial state x0
                        begin_node = self.graph[i][j]
                        for node in self.graph[i+1]:
                            if not (abs(node.x - self.light.location) > 1 and node.v == 0):
                                # print(node.x, node.v)
                                delta_v = node.v - begin_node.v
                                # if begin_node.v + node.v > 0:
                                # this is to exclude the velocity tranfer 0 - 0 velocity happens
                                delta_t = 2 * self.car.delta_x/(node.v + begin_node.v)
                                a = delta_v / delta_t
                                if a < self.car.a_max and a > self.car.a_min:
                                    j = delta_t * self.car.w1 / self.car.delta_t_min + \
                                        abs(a/self.car.a_max) * self.car.w2
                                    edge = {"v1": begin_node, "v2": node,
                                            "delta_t": delta_t, "cost": j, "a": a}
                                    node.income_edge.append(edge)
                                    begin_node.outcome_edge.append(edge)

    def solver(self, dv):
        # first make the grid states
        start_loc, end_loc = self.car.x0[0], self.light.location + self.car.delta_x
        loc_num = math.floor((end_loc - start_loc)/self.car.delta_x)
        vel_num = math.floor(self.car.v_max/dv) + 1
        loc_grid = np.linspace(start_loc + self.car.delta_x, end_loc, loc_num)
        vel_grid = np.linspace(0, self.car.v_max, vel_num)

        # build the graph
        self.build_graph(loc_grid, vel_grid)

        sol = {"t": 0, "cost": 0, "action": []}
        Total_solutions = []  # stores tuples of [cost, tuple] combos for all status
        # red light optimal sol
        self.DFS_red_light(self.graph[loc_num - 2][0], sol)
        # pick up the solution with lowest cost for this state's best solution
        if self.sol != []:
            solution = Sort_Tuple(self.sol)[0]
            solution.insert(0, 0)

            Total_solutions.append(solution)
            self.sol = []
        for i in range(1, len(self.graph[loc_num - 2])):

            sol = {"t": 0, "cost": 0, "action": []}
            self.DFS_green_light(self.graph[loc_num - 2][i], sol)
            # pick up the solution with lowest cost for this state's best solution
            if self.sol != []:
                solution = Sort_Tuple(self.sol)[0]
                solution.insert(0, i)

                Total_solutions.append(solution)
                self.sol = []

        # in Total_solutions, the form is [index of the status in graph[-2], cost, action]

        # after we know the all optimal solutions in front of traffic light, then we
        # move on to discover the step after passing the light, the final step
        Final_solution = []
        for i in range(len(self.graph[-1])):
            if self.graph[-1][i].v != 0:
                node = self.graph[-1][i]
                # Dynamic Programming in assuring the last step in traffic light
                for pre_solution in Total_solutions:
                    begin_node = self.graph[-2][pre_solution[0]]
                    delta_v = node.v - begin_node.v
                    delta_t = 2 * self.car.delta_x/(node.v + begin_node.v)
                    a = delta_v / delta_t
                    if a < self.car.a_max and a > self.car.a_min:
                        j = delta_t * self.car.w1 / self.car.delta_t_min + \
                            abs(a/self.car.a_max) * self.car.w2
                        if j + pre_solution[1] < node.cost:

                            node.cost = j + pre_solution[1]
                            node.action = copy.deepcopy(pre_solution[2])

                            node.action.append(a)

                Final_solution.append([node.cost, node.action])
        # print(Final_solution)
        optimal_solution = Sort_Tuple(Final_solution)[0]
        self.optimal_control = optimal_solution[1]

        # calculate the velocity for every grid states
        self.cal_opt_vel()

        return self.optimal_control, optimal_solution[0], self.record

    def cal_opt_vel(self):
        self.record = [self.car.x0[1]]
        for i in range(len(self.optimal_control)):
            a = self.optimal_control[i]
            self.record.append(np.sqrt(self.car.delta_x*2*a + self.record[i]**2))

    def feedback_controller(self, x, index):
        a = self.optimal_control[index]
        delta_x = (x[0] - self.car.x0[0]) % self.car.delta_x
        k = .5
        v_ref = np.sqrt(self.record[index]**2 + 2*delta_x*a)
        delta_v = v_ref - x[1]
        return k*delta_v

    def controller(self, x):
        if self.go_after_redlight:
            if x[0] < self.light.location + self.car.delta_x:
                return self.optimal_control[-1]
        if x[0] < self.light.location:
            if x[2] != 3:
                # if x[3] == 3 and self.light.location - x[0] < 0.5 and x[1] == 0:
                if x[3] == 3 and self.stop_at_redlight:
                    self.go_after_redlight = True
                    return self.optimal_control[-1]
                index = math.floor((x[0] - self.car.x0[0])/self.car.delta_x)
                # print(x[0], self.car.x0[0], self.car.delta_x, index)
                return self.optimal_control[index] + self.feedback_controller(x, index)
            elif x[3] == 3 and self.light.location - x[0] < 0.5 and abs(x[1]) < 0.5:
                self.stop_at_redlight = True
                return 'stop'
            else:
                index = math.floor((x[0] - self.car.x0[0])/self.car.delta_x)
                # print(x[0], self.car.x0[0], self.car.delta_x, index)
                return self.optimal_control[index] + self.feedback_controller(x, index)

        else:
            return 0

    def dynamics(self, s, u):
        #     state of the system
        #     s_k = (y, v, l, l_p)
        #     s_{k+1} = f(s_k, u_k)

        if u == 'stop':
            vel = 0
            location = s[0]
        else:

            location = s[0] + u * self.car.delta_t ** 2 / 2 + s[1] * self.car.delta_t
            vel = s[1] + u * self.car.delta_t
        lp = s[2]
        l = lp
        return [location, vel, l, lp]
