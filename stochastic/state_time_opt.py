import copy
import math
import numpy as np
from util import opt_vehicle, traffic_light
from fuel_eco import cost_fuel
import os
import pandas as pd

class state_space:
    '''
    a time-state searching method on deterministic part of traffic signal
    '''

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
        self.init_state = []
        # self.w1 = 0.2   # work to do against friction
        # self.w2 = 0.1   # idling cost
        # self.w3 = 0.6   # accelerating cost weight
        # self.alpha = 0.6289   # idling cost cm^3/s
        self.x_base = 0
        self.t_base = 0
        self.v_base = 0
        
    def build_graph(self):
        
    
    def solver(self, x, v, dv):
        # we need to know the initial position, then directly optimize it
        self.x_base = x
        # self.t_base = 0
        self.v_base = v