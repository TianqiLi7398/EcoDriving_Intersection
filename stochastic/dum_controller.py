# this is created in 2019/04/07 for a simple simulation of traffic intersection crossing
# Author: Tianqi Li

import numpy as np
import math




class dummy_control:

    def __init__(self, light, car):
        self.light = light
        self.car = car
        self.rush_yellow = 0
        self.opt_vel = 12
        self.stop_at_red = False


    def controller(self, s):

        k = 0.1
        if s[0] < self.light.location:
            # in front of the intersection
            if s[2] != 1:
                if self.stop_at_red:
                    return 0

                 # not green light, calculate the acceleration to stop at intersection
                a =  max(self.car.a_min, - (s[1]) ** 2 / (2 * (self.light.location - s[0])))
                if self.light.location - s[0] < 0.5:
                    a = - s[1]/ self.car.delta_t + 0.01
                    self.stop_at_red = True
                return a
            else:
                return max(self.car.a_min, min(self.car.a_max, (self.opt_vel - s[1]) / self.car.delta_t))
        else:
            return max( self.car.a_min, min( (self.opt_vel - s[1]) / self.car.delta_t, self.car.a_max))

    def dynamics(self, s, u):
    #     state of the system
    #     s_k = (y, v, l, l_p)
    #     s_{k+1} = f(s_k, u_k)
        y = s[0] + u * self.car.delta_t **2 / 2 + s[1]*self.car.delta_t
        v = s[1] + u * self.car.delta_t
        lp = s[2]
        l = lp
        return [y, v, l, lp]
