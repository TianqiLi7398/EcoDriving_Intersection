# this is created in 2020/06/29 for a simple simulation of traffic intersection crossing
# Author: Tianqi Li

import numpy as np
import math




class smart_control:

    def __init__(self, light, car, a_max, a_min, opt_vel=12):
        self.light = light
        self.car = car
        self.rush_yellow = 0
        self.opt_vel = opt_vel
        self.stop_at_red = False
        self.a_max = a_max 
        self.a_min = a_min 
        self.idling = False
        self.see_green = False

    def chase_opt_speed(self, s):
        return max(self.car.a_min, min(self.car.a_max, (self.opt_vel - s[1]) / self.car.delta_t))

    def anticipate_accident(self, s, a):
        # anticipate 2 cases for vehicle drive in green light
        # rush within yellow light
        dt = self.light.yel_dur
        v = s[1] + dt * a
        x = s[0] + s[1] * dt + 0.5 * a * (dt **2)
        l = self.light.location - x
        if l < 0:
            return False
        # case 1, rush within yellow light using max acc 
        t1 = (self.car.v_max - v) / self.a_max 
        if t1 < dt:
            l_rush = t1 * (v + 0.5 * self.a_max * t1) + (dt - t1) * self.car.v_max
        else:
            l_rush = dt * (v + 0.5 * self.a_max * dt)
        # if distance to intersection is able to rush within yellow light, no accident
        if l < l_rush:
            return False
        # case 2, stop before red light 
        t2 = - v / self.a_min 
        l_stop = v * t2 * 0.5
        if l > l_stop:
            return False
        else:
            return True

    def adjust_acc(self, v_aim, v, l):
        dv = v_aim - v
        if np.isclose(dv, 0):
            return 0
        return self.replan(l, dv, v)

    def replan(self, dx, dv, v):
        
        dt = dx / (v + dv * 0.5)
        a = dv / dt
        if a > self.car.a_max:
            a = self.car.a_max
        elif a < self.car.a_min:
            a = self.car.a_min
        return a
        

    def controller(self, s):
        
        if s[0] > self.light.location:
            a = self.chase_opt_speed(s)
        else:
            if s[2] == 1:
                self.see_green = True
                if s[-1] == 3 and self.light.location - s[0] < 0.5 and self.light.location > s[0]:
                    a = self.chase_opt_speed(s)
                else:
                    # green light, need to worry about the future's red light
                    a = self.chase_opt_speed(s)
                    if self.anticipate_accident(s, a):
                        # if anticipate accident, it would be better to slow down
                        a = self.a_min
            elif s[2] == 3:
                # red light, need to stop anyway

                if self.light.location - s[0] < 0.5 and self.light.location > s[0] and s[1] < 1:
                    a = 'stop'
                # calculate the acc 
                else:
                    a = self.adjust_acc(0, s[1], self.light.location - s[0])
                            
            else:
                # yellow light, decide rush or stop, actually you need to know if you start with yellow/did you see green light before.
                # if you've seen green bofore, you know you have full time to go through the traffic light;'
                # other wise, plan a safe stop
                # case 1, rush within yellow light using max acc 
                if self.see_green:
                    dt = self.light.yel_dur
                    l = self.light.location - s[0]
                    t1 = (self.car.v_max - s[1]) / self.a_max 
                    if t1 < dt:
                        l_rush = t1 * (s[1] + 0.5 * self.a_max * t1) + (dt - t1) * self.car.v_max
                    else:
                        l_rush = dt * (s[1] + 0.5 * self.a_max * dt)
                    if l < l_rush:
                        if np.isclose(self.car.v_max, s[1]):
                            a = 0 
                        else:
                            a = self.a_max
                    else:
                        # case 2, stop at the intersection
                        a = self.adjust_acc(0, s[1], l - 0.2)
                else:
                    # just plan for stop
                    a = self.adjust_acc(0, s[1], self.light.location - s[0])
        return a

    def dynamics(self, s, u):
    #     state of the system
    #     s_k = (y, v, l, l_p)
    #     s_{k+1} = f(s_k, u_k)
        if u == 'stop':
            self.idling = True
            vel = 0
            location = s[0]
        else:
            location = s[0] + u * self.car.delta_t ** 2 / 2 + s[1] * self.car.delta_t
            vel = s[1] + u * self.car.delta_t
            if vel > self.car.v_max:
                if vel - self.car.v_max < 0.5:
                    vel = self.car.v_max
                else:
                    print("alert for overspeeding!!!")
        lp = s[2]
        l = lp ### l will be updated by external light class
        return [location, vel, l, lp]
