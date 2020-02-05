import math
import numpy as np


class traffic_light:
    def __init__(self, t0, timeset, location):
        self.t0 = t0
        self.red_dur = timeset[0]
        self.yel_dur = timeset[1]
        self.gre_dur = timeset[2]
        self.T = self.red_dur + self.yel_dur + self.gre_dur
        self.location = location
        self.tg = self.gre_dur
        # here regard yellow light as red light
        self.tr = self.red_dur + self.yel_dur
        # here k defines the traffic light is the unkonw or unknown pattern
        k = 0
        self.red_dur += np.random.randn(1) * k
        self.gre_dur += np.random.randn(1) * k

    def give_clock(self, t):
        t = (t + self.t0) % self.T
        if t < self.gre_dur:
            #             green light
            return 1
        elif t < self.gre_dur + self.yel_dur:
            #             yellow light
            return 2
        else:
            # red light
            return 3

    def trafficline(self, final_time):
        # returns the signals time interval + location combos of the problem
        green, yellow, red = [], [], []
        t = self.t0

        if t < self.gre_dur:
            # green light
            green = [[0, self.location], [self.gre_dur - t, self.location]]
            yellow = [[self.gre_dur - t, self.location],
                      [self.gre_dur - t + self.yel_dur, self.location]]
            red = [[self.gre_dur - t + self.yel_dur, self.location], [self.T - t, self.location]]
        elif t < self.gre_dur + self.yel_dur:
            # yellow light
            yellow = [[0, self.location], [self.gre_dur + self.yel_dur - t, self.location]]
            red = [[self.gre_dur + self.yel_dur - t, self.location],
                   [self.T - t, self.location]]
            # green = [[self.yel_dur + self.red_dur - t, self.location], [self.T - t, self.location]]

        else:
            # red light
            red = [[0, self.location], [self.T - t, self.location]]
            # green = [[self.red_dur - t, self.location],
            #          [self.red_dur - self.gre_dur - t, self.location]]
            # yellow = [[self.red_dur - self.gre_dur - t, self.location], [self.T - t, self.location]]

        final_time -= (self.T - self.t0)
        t = self.T - self.t0

        while final_time > 0:
            if final_time > self.gre_dur:
                green.append([t, self.location])
                green.append([t + self.gre_dur, self.location])
            else:
                green.append([t, self.location])
                green.append([t + final_time, self.location])
            t += self.gre_dur
            final_time -= self.gre_dur

            if final_time > self.yel_dur:
                yellow.append([t, self.location])
                yellow.append([t + self.yel_dur, self.location])
            else:
                yellow.append([t, self.location])
                yellow.append([t + final_time, self.location])
            t += self.yel_dur
            final_time -= self.yel_dur

            if final_time > self.red_dur:
                red.append([t, self.location])
                red.append([t + self.red_dur, self.location])
            else:
                red.append([t, self.location])
                red.append([t + final_time, self.location])
            t += self.red_dur
            final_time -= self.red_dur

        return red, green, yellow

    def prediction(self, l, tp):
        #         return the conditional probability of [p[l(t + tp) = g/l(t) = l], p[l(t + tp) = r/l(t) = l]]
        tm = tp % self.T

        if l == 1:
            if self.tr < self.tg:
                if tm <= self.tr:
                    p = (self.tg - tm)/self.tg
                elif tm > self.tr and tm <= self.tg:
                    p = (self.tg - self.tr)/self.tg
                else:
                    p = (tm - self.tr)/self.tg
            else:
                if tm <= self.tg:
                    p = (self.tg - tm)/self.tg
                elif tm > self.tg and tm <= self.tr:
                    p = 0
                else:
                    p = (tm - self.tr)/self.tg
        else:
            if self.tr < self.tg:
                if tm <= self.tr:
                    p = tm/self.tr
                elif tm > self.tr and tm <= self.tg:
                    p = 1
                else:
                    p = (self.tg + self.tr - tm) / self.tr
            else:
                if tm <= self.tg:
                    p = tm/self.tr
                elif tm > self.tg and tm <= self.tr:
                    p = self.tg/self.tr
                else:
                    p = (self.tg + self.tr - tm) / self.tr

        return [p, 1-p]

    def est_waiting(self, l, tp):
        tm = tp % self.T
        if l == 1:
            if self.tg < self.tr:
                if tm <= self.tg:
                    e = self.tr - tm/2
                elif self.tg < tm and self.tr >= tm:
                    e = self.tr - tm + self.tg/2
                else:
                    e = (self.T - tm)/2
            else:
                if tm <= self.tr:
                    e = self.tr - tm/2
                elif tm > self.tr and tm <= self.tg:
                    e = self.tr/2
                else:
                    e = (self.T - tm)/2
        else:
            if tm <= self.tr:
                e = (self.tr - tm)/2
            elif tm > self.tg:
                e = self.tr - (tm - self.tg)/2
            else:
                # this is the impossible scene
                e = -1
        return e


class vehicle:
    def __init__(self, location, vel, v_opt, v_max, dt):
        self.v_opt = v_opt
        self.v_max = v_max
        self.dt = dt
        self.location = location
        self.vel = vel

    def controller(self, s, light_num, light_location):

        k = 0.1
        if s[0] < light_location[light_num]:
            # in front of the intersection
            if s[2] != 1:
                 # not green light, calculate the acceleration to stop at intersection
                a = - s[1] ** 2 / (2 * (light_location[light_num] - s[0]))
                if abs(light_location[light_num] - s[0]) < 0.3:
                    a = - s[1] / self.dt + 0.01
                return a
            else:
                # green light, a propotional control to keep in the v_opt
                return min(1, (self.v_opt - s[1]) / self.dt)
        else:
            return 0

    def dynamics(self, s, u):
        #     state of the system
        #     s_k = (y, v, l, l_p)
        #     s_{k+1} = f(s_k, u_k)
        self.location = s[0] + u * self.dt ** 2 / 2 + s[1] * self.dt
        self.vel = s[1] + u * self.dt
        lp = s[2]
        l = lp
        return [self.location, self.vel, l, lp]

    def inter_ahead(self, location):
        for i in range(len(location)):
            if self.location < location[i]:
                return i
        return -1


class opt_vehicle:
    def __init__(self, delta_x, v_max, a_max, a_min, light, x0):
        self.delta_x = delta_x
        self.epslon = 10 ** -6
        self.v_max = v_max
        self.delta_t_min = delta_x / v_max
        self.a_max = a_max
        self.a_min = a_min
        self.w1, self.w2 = 1/8, 1/8
        self.light = light
        self.x0 = x0
        self.delta_t = 0.01

    def dynamics(self, s, u):
        #     state of the system
        #     s_k = (y, v, l, l_p)
        #     s_{k+1} = f(s_k, u_k)
        location = s[0] + u * self.dt ** 2 / 2 + s[1] * self.dt
        vel = s[1] + u * self.dt
        lp = s[2]
        l = lp
        return [location, vel, l, lp]

    def optimal_controller(self, x):
        # TODO remains correction
        if x[0] < 50:
            index = math.floor(x[0]/10)
            return opt_sol[index]
        else:
            return 0
