import abc
import math
import numpy as np
from initial import *

def main():
    init = Initial()
    T_to_hit = init.get_tth()
    interval = init.get_intvl()
    all_ac = init.get_acs()
    step = 0
    while step < interval:
        #TODO0: define set_order()
        order = set_order(all_ac, T_to_hit)
        for ac in order:
            #TODO1: define slv_mdp
            trvl_plan = slv_mdp()
            all_ac[ac][2] = trvl_plan #update travel travel plan

def cost(ac_name, all_ac, T_to_hit, step, own_height, action):
    intruder = dict()
    intruders_plan = list()
    i = 0
    for key in all_ac:
        if key == ac_name:
            hit_point = T_to_hit[i]
            del hit_point[i]
            continue
        else:
            intruder[key] =  all_ac[key]
            intruders_plan.append(intruder[key][2])
            i += 1
    intruders_plan = np.transpose(np.array(intruders_plan))
    relative_height = own_height - intruders_plan[step]
    state_cost = np.sum(np.exp(-abs(relative_height/185)) * np.exp(-abs(hit_point/60)))
    if action == 2500 or action == -2500:
        action_cost = 0.002
    elif action == 1500 or action == -1500:
        action_cost = 0.001
    elif action == 0:
        action_cost = 0
    else:
        print('wrong action')
    current_cost = action_cost + state_cost
    return current_cost

#TODO0:
def set_order(all_ac, T_to_hit):
    pass
    #call cost()
    #return ac_name = ['ac1', 'ac2', 'ac3']
#TODO1:
def slv_mdp():
    pass






def get_state(ac_name, all_ac, T_to_hit):
    own, intruder = [], []
    i, j = 0, 0
    for key in all_ac:
        if key == ac_name:
            own = all_ac[ac_name]
            own_T = T_to_hit[j]
        else:
            intruder[i] = all_ac[ac_name]
            i += 1
        j += 1
    state = np.array(intruder) - np.array(own)
    state = np.vstack((state, own_T))
    state = np.transpose(state)
    return state


class ModelState:
    def __init__(self, state, T0):
        self.relative_height = state[0]
        self.relative_velocity = state[1]   #ft/min
        self.tau = state[2]
        self.X_SPEED = 1
        self.T = T0

    def state_discrect(self):
        h, h_dot = [], []
        for i in range(len(self.relative_height)):
            if self.relative_height[i] > 1000 or self.relative_height[i] < -1000:
                relative_height = np.sign(self.relative_height[i]) * 1000
            else:
                relative_height = self.relative_height[i]
            h.append(np.sign(relative_height) * int(abs(relative_height)/100) * 100)
        for j in range(len(self.relative_velocity)):
            if self.relative_velocity[j] > 5000 or self.relative_velocity[j] < -5000:
                relative_velocity = np.sign(self.relative_velocity[j]) * 5000
            else: relative_velocity = self.relative_velocity[j]
            h_dot.append(np.sign(relative_velocity) * int(abs(relative_velocity)/250) * 250)
        tau = self.tau
        state_discrect = [h, h_dot, tau]
        return state_discrect


    def hit_position(self):
        x = self.T * self.X_SPEED
        y = self.relative_height + self.relative_velocity/60 * self.tau
        hit_point = [x, y]
        return hit_point

    def updated_state(self, action):  # the updated state is observed through sensors in practice
        self.relative_velocity = action * np.ones(len(self.relative_velocity), dtype = int)
        self.relative_height = self.relative_height + self.relative_velocity/60
        self.tau = np.array(self.tau) - np.array(np.ones(len(self.tau), dtype = int))
        next_state = self.state_discrect()
        return next_state

    def cost(self, action):
        hit_point = self.hit_position()
        intruder_cost = []
        for i in range(len(hit_point[1])):
            if abs(hit_point[1][i]) >= 850:
                intruder_cost.append(-0.0001)
            else:
                intruder_cost.append(np.exp(hit_point[1][i]/185))  # ==>need weighting: 850/185 = ln(0.01)
        state_cost = min(self.tau/60) * sum(intruder_cost)
        if action == 2500 or action == -2500:
            action_cost = 0.002
        elif action == 1500 or action == -1500:
            action_cost = 0.001
        elif action == 0:
            action_cost = 0
        else:
            print('wrong action')
        current_cost = state_cost + action_cost
        return current_cost


if __name__ == "__main__":
    main()
    """md = ModelState([[190, 100], [700, -500], [40, 60]], [40, 60])
    print(md.relative_height)
    print(md.state_discrect())
    next_st = md.updated_state(2500)
    next_next_st = md.updated_state(2500)
    print(next_next_st)
    print(md.hit_position())
    print(md.cost(2500))"""
